#!/usr/bin/env python
#
# imuserial - external IMU sample reader for pypilot
#
# Streams raw accel / gyro / compass samples from a microcontroller over a USB
# serial (CDC) connection and hands them to RTIMULib via RTIMU.setExtIMUData().
#
# Wire protocol (ASCII, LF or CR/LF terminated, one sample per line):
#
#   seq,ts_us,ax,ay,az,gx,gy,gz,mx,my,mz\n      (11 columns, current)
#   seq,ax,ay,az,gx,gy,gz,mx,my,mz\n            (10 columns, legacy)
#
#   seq    : 16 bit counter 0..65535 (wraps around); used to detect resets
#   ts_us  : 32 bit micros() taken right after the sensor read completed
#            (~71.5 min wraparound).  The host converts the delta between
#            successive samples into the true sample interval dt.
#   ax..az : accelerometer [g]
#   gx..gz : gyroscope       [rad/s]
#   mx..mz : magnetometer    [uT]
#
# For the 11-column format each sample carries the device micros() at sample
# completion, so the timestamp here is integrated as timestamp += dt where
# dt is the unsigned 32 bit delta of ts_us between consecutive samples (this
# handles wraparound automatically).  USB batching delays delivery but never
# reorders samples, so the RTIMULib fusion (RTQF / Kalman4 use timestamp
# *deltas* for gyro integration) stays on the true sample timeline even when
# the device skips transmissions or the host is briefly delayed.
#
# The legacy 10-column format has no device clock; those samples are stamped
# with the nominal sample period on a strict monotonic clock.
#
# In both cases the clock is re-synchronized to the current wall-clock
# whenever the stream stalls or restarts, and ts_us jumps impossibly
# (dt == 0 or dt > 1 s).  Malformed / out of range lines are dropped
# without interrupting the stream.

import threading
import time
from collections import deque

try:
    import serial
except Exception:
    serial = False

STALE_TIMEOUT = 1.0    # seconds without a valid sample -> stream considered dead
MAX_QUEUED = 400       # backlog bound when the consumer is slow (4 s @ 100 Hz)

# sanity limits, lines with values outside are rejected as corruption
MAX_ACCEL = 4.0        # g
MAX_GYRO = 35.0        # rad/s  (>2000 deg/s is impossible from an ICM-20948)
MAX_MAG = 1000.0       # uT     (earth field is ~25-65 uT)

class IMUSerialReader(object):
    def __init__(self, port, baud=115200, sample_rate=100.0):
        if not serial:
            raise Exception('pyserial module not available')
        self.port = port
        self.baud = int(baud)
        self.sample_rate = max(1.0, float(sample_rate))
        self.period_us = 1e6 / self.sample_rate

        self._queue = deque()
        self._mutex = threading.Lock()
        self._thread = False
        self._running = False
        self._last_seq = False   # False = no sample received yet
        self._last_xiao_ts = None  # last device ts_us (11-column format)
        self._last_ts = 0
        self._last_valid = 0.0   # monotonic time of last accepted sample
        self._open_error = ''

    # ------------------------------------------------------------------
    # control
    # ------------------------------------------------------------------
    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._process, name='imuserial', daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        thread, self._thread = self._thread, False
        if thread:
            thread.join(2)
        with self._mutex:
            self._queue.clear()
            self._last_seq = False
            self._last_xiao_ts = None
            self._last_ts = 0
            self._last_valid = 0.0

    # ------------------------------------------------------------------
    # consumer side (called from the pypilot imu process)
    # ------------------------------------------------------------------
    def drain(self):
        '''return and clear all samples received since the last call'''
        with self._mutex:
            samples = list(self._queue)
            self._queue.clear()
        return samples

    def fresh(self):
        '''true if there is pending data or the stream was alive recently'''
        with self._mutex:
            if self._queue:
                return True
            if not self._last_valid:
                return False
            return time.monotonic() - self._last_valid <= STALE_TIMEOUT

    def error(self):
        return self._open_error

    # ------------------------------------------------------------------
    # serial side
    # ------------------------------------------------------------------
    def _process(self):
        while self._running:
            try:
                ser = serial.Serial(self.port, self.baud, timeout=0.2)
            except Exception as e:
                self._open_error = str(e)
                self._sleep(1.0)  # keep retrying the open
                continue

            self._open_error = ''
            pending = b''
            last_line = time.monotonic()
            try:
                while self._running:
                    try:
                        chunk = ser.read(512)
                    except Exception:
                        # transient zero-read / spurious ready flag seen on
                        # some CDC devices: harmless while data keeps flowing,
                        # so ignore it.  Only reopen when the stream stalls.
                        if time.monotonic() - last_line > 3.0:
                            self._open_error = 'no data received for 3 seconds'
                            break
                        self._sleep(0.02)
                        continue

                    if not chunk:
                        # normal read timeout; reopen if the stream has been
                        # silent (device hung but USB is still enumerated)
                        if time.monotonic() - last_line > 3.0:
                            self._open_error = 'no data received for 3 seconds'
                            break
                        continue

                    last_line = time.monotonic()
                    pending += chunk
                    while b'\n' in pending:
                        line, pending = pending.split(b'\n', 1)
                        text = line.decode('ascii', 'replace').strip()
                        if text:
                            self._open_error = ''
                            self._handle(text)
            except Exception as e:
                self._open_error = str(e)
            finally:
                try:
                    ser.close()
                except Exception:
                    pass
            self._sleep(0.5)

    def _sleep(self, seconds):
        '''interruptible sleep used while waiting to reopen the device'''
        end = time.monotonic() + seconds
        while self._running:
            remaining = end - time.monotonic()
            if remaining <= 0:
                return
            time.sleep(min(0.1, remaining))

    def _handle(self, text):
        fields = text.split(',')
        if len(fields) not in (10, 11):
            return
        try:
            seq = int(fields[0]) & 0xffff
            if len(fields) == 11:
                xiao_ts = int(fields[1]) & 0xffffffff
                ax, ay, az = float(fields[2]), float(fields[3]), float(fields[4])
                gx, gy, gz = float(fields[5]), float(fields[6]), float(fields[7])
                mx, my, mz = float(fields[8]), float(fields[9]), float(fields[10])
            else:
                xiao_ts = None
                ax, ay, az = float(fields[1]), float(fields[2]), float(fields[3])
                gx, gy, gz = float(fields[4]), float(fields[5]), float(fields[6])
                mx, my, mz = float(fields[7]), float(fields[8]), float(fields[9])
        except ValueError:
            return

        if not _in_range((ax, ay, az), MAX_ACCEL) or \
           not _in_range((gx, gy, gz), MAX_GYRO) or \
           not _in_range((mx, my, mz), MAX_MAG):
            return  # corrupted line, drop silently

        now = time.monotonic()
        with self._mutex:
            if self._last_seq is not False:
                # a large backward jump means the far side restarted; discard
                # any backlog so old and new samples are never fused together
                forward = (seq - self._last_seq) & 0xffff
                if forward == 0 or forward >= 0x8000:
                    self._queue.clear()
                    self._last_ts = 0
                    self._last_xiao_ts = None
            self._last_seq = seq

            # timestamp synthesis (see module docstring)
            dt = None
            if xiao_ts is not None and self._last_xiao_ts is not None:
                # device real sample interval (unsigned 32 bit: wrap safe)
                dt = (xiao_ts - self._last_xiao_ts) & 0xffffffff
                if dt == 0 or dt > 1000000:  # 1 s jump: restart/stall
                    dt = None
            if self._last_ts and dt is not None:
                timestamp = self._last_ts + dt
            elif self._last_ts and xiao_ts is None and \
                    (now - self._last_valid) <= 3 * self.period_us / 1e6:
                # legacy device without ts_us: nominal period
                timestamp = self._last_ts + self.period_us
            else:
                timestamp = int(now * 1e6)  # stream stalled/restarted: resync
            self._last_ts = timestamp
            self._last_xiao_ts = xiao_ts
            self._last_valid = now

            if len(self._queue) >= MAX_QUEUED:
                self._queue.popleft()  # keep the newest samples, drop the oldest
            self._queue.append((gx, gy, gz, ax, ay, az, mx, my, mz, int(timestamp)))


def _in_range(values, limit):
    for v in values:
        if not (-limit <= v <= limit):
            return False
    return True


if __name__ == '__main__':
    # simple offline self test of the parse / timestamp logic
    reader = IMUSerialReader('/dev/null', 115200, 100.0)
    t0 = time.monotonic()
    for i in range(25):
        # simulate a 100 Hz stream: 10 ms apart in wall time
        while time.monotonic() - t0 < (i + 1) * 0.01:
            time.sleep(0.001)
        reader._handle('%d,0,0,1,0,0,0,30,0,40' % (i & 0xffff))
    samples = reader.drain()
    print('received', len(samples), 'samples')
    assert samples
    assert all(len(s) == 10 for s in samples)
    ts = [s[9] for s in samples]
    # timestamps must be strictly increasing, ~10 ms apart
    for a, b in zip(ts, ts[1:]):
        assert b - a > 0, 'timestamp not strictly increasing'
    assert abs((ts[-1] - ts[0]) / 1e6 - 0.24) < 0.02, 'timestamp drift'
    # a device restart (seq reset to 0) must discard the stale backlog and
    # re-synchronize the timestamp clock to the current time
    time.sleep(0.05)   # let wall clock move past the synthetic timestamps
    now_before = int(time.monotonic() * 1e6)
    reader._handle('0,0,0,1,0,0,0,30,0,40')
    restarted = reader.drain()
    assert len(restarted) == 1
    assert restarted[0][9] > ts[-1]            # strictly increasing
    assert restarted[0][9] >= now_before       # clock jumped to ~now after reset
    # corrupted lines must be ignored
    reader._handle('garbage,line,without,enough,fields')
    reader._handle('1,9999,0,1,0,0,0,30,0,40')   # accel out of range
    assert reader.drain() == []

    # ---- 11-column (device ts_us / dynamic dt) stream ----
    reader2 = IMUSerialReader('/dev/null', 115200, 100.0)
    base = 1000000000  # arbitrary device uptime [us]
    t0 = time.monotonic()
    for i in range(25):
        # simulate a 100 Hz stream: 10 ms apart in wall time
        while time.monotonic() - t0 < (i + 1) * 0.01:
            time.sleep(0.001)
        reader2._handle('%d,%d,0,0,1,0,0,0,30,0,40' % (i & 0xffff, base + i * 10000))
    s2 = reader2.drain()
    assert len(s2) == 25
    assert all(len(s) == 10 for s in s2)   # queued tuples keep the same shape
    ts2 = [s[9] for s in s2]
    for a, b in zip(ts2, ts2[1:]):
        assert b - a == 10000, '11-col dt broken: %d' % (b - a)

    # ts_us 32 bit wraparound must still yield the correct dt
    reader2._handle('25,%d,0,0,1,0,0,0,30,0,40' % 0xfffffff0)
    reader2._handle('26,%d,0,0,1,0,0,0,30,0,40' % ((0xfffffff0 + 10000) & 0xffffffff))
    wrap = reader2.drain()
    assert len(wrap) == 2
    assert wrap[1][9] - wrap[0][9] == 10000, 'wrap dt broken'

    # a dt jump > 1 s (device stalled without seq reset) resyncs to wall clock
    hi = 0xffffffff - 500
    reader2._handle('27,%d,0,0,1,0,0,0,30,0,40' % hi)
    reader2._handle('28,%d,0,0,1,0,0,0,30,0,40' % ((hi + 10000) & 0xffffffff))
    jump = reader2.drain()
    assert len(jump) == 2
    # first of the pair resynced, second continues +10 ms
    assert jump[1][9] - jump[0][9] == 10000, 'post-resync dt broken'

    print('imuserial self test OK')
