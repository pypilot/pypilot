import time
import spidev


spi = spidev.SpiDev()
spi.open(0, 1)
spi.max_speed_hz=100000

rnds = 10

totallatency = 0
for i in range(rnds):
    spi.xfer([0, 0, 0, 0])
    t = time.time()
    while True:
        x = spi.xfer([1])
        if x[0]:
            break

    totallatency += time.time() - t
    spi.xfer([0, 0, 0, 0])
    time.sleep(.01)

spi.close()

print 'spi latency', totallatency/rnds


import serial
device = serial.Serial('/dev/ttyUSB0', 38400)
device.timeout=0
totallatency = 0
for i in range(rnds):
    device.write('XXXXXXXXXXXX')
    time.sleep(.1)
    while True:
        x = device.read(1)
        if  not  x:
            break

#    time.sleep(.01)
    t = time.time()
    while True:
        device.write('%c' % i);
        x = device.read(1)
        if x and ord(x) == i:
            break
#        if x:
#            print ('need more sleeps', x)
        
    totallatency += time.time() - t

    time.sleep(.01)

device.close()


print 'uart latency', totallatency/rnds
