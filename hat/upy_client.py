##  python3 -m mpy_cross -v -march=xtensawin testmodule.py 

import wifi_esp32


import socket, time, json

DEFAULT_PORT = 23322

class pypilotClient(object):
    def __init__(self, host=False):
        self.connection = False
        self.host = host
        self.watches = {}
        self.wwatches = {}
        self.values = {}

    def disconnect(self):
        if not self.connection:
            return

        self.connection.close()
        self.connection = False
        
    def connect(self):
        if self.connection or not self.host:
            return
    
        addr_info = socket.getaddrinfo(self.host, DEFAULT_PORT)
        addr = addr_info[0][-1]
        self.connection = socket.socket()
        for name, value in self.watches.items():
            self.wwatches[name] = value # resend watches
        self.wwatches['values'] = True # watch values
        self.values = {}
        self.connection.settimeout(0)
        import uselect
        self.poller = uselect.poll()
        self.poller.register(self.connection, uselect.POLLIN)
        
        print('connect to pypilot....')
        try:
            self.connection.connect(addr)
            self.connection.settimeout(0)
        except OSError as e:
            import errno
            if e.args[0] not in [errno.EINPROGRESS, errno.ETIMEDOUT]:
                print('failed to connect', e)
                import time
                time.sleep(.25)
                self.disconnect()

    def receive(self):
        if not self.connection:
            self.connect()
            return {}

        # inform server of any watches we have changed
        if self.wwatches:
            self.set('watch', self.wwatches)
            self.wwatches = {}

        msgs = {}
        while True:
            try:
                if not self.poller.poll(0):
                    break
                line = self.connection.readline()
                if not line:
                    break
                line = line.decode()
                #print('line', line)
                name, value = line.split('=', 1)
                if name == 'values':
                    for n, v in value.items:
                        self.values[n] = v
                elif name == 'error':
                    print('server error:', value)
                else:
                    msgs[name] = json.loads(value.rstrip())
            except Exception as e:
                print('failed read line', e)
        return msgs

    def get_values(self):
        return self.values

    def list_values(self, timeout=0):
        return {}
    
    def watch(self, name, period=True):
        if name in self.watches and self.watches[name] is period:
            return
        self.wwatches[name] = period
        self.watches[name] = period
        
    def set(self, name, value):
        if not self.connection:
            return
        try:
            line = json.dumps(value)+'\n'
            self.connection.send(name + '=' + line)
        except Exception as e:
            print('failed to watch', e)
            self.connection = False

def main():
    essid, psk = 'openplotter', '12345678'
    
    import network
    station = network.WLAN(network.STA_IF)  # client, not AP    
    client = pypilotClient('10.10.10.1')
    client.watch('imu.heading') # fastest rate
    client.watch('ap.heading', 1) # once per second

    while True:
        if not station.isconnected():
            print('connection', station.isconnected(), station.status())
            station.active(True) # enable wifi
            station.connect(essid, psk)
            for x in range(100):
                if station.isconnected():
                    print('connection success', station.ifconfig())
                    break
                time.sleep(.1)
            else:
                print('failed to connect to', essid)
                time.sleep(3)
            continue
        
        msgs = client.receive()
        if not msgs:
            time.sleep(.1)

        for name, value in msgs.items():
            print(name, '=', value)
            
if __name__ == '__main__':
    main() 

