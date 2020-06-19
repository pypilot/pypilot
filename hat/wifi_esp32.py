import network

essid, psk = 'openplotter', '12345678'

station = network.WLAN(network.STA_IF)  # client, not AP
station.active(True) # enable wifi
station.connect(essid, psk)
connected = False
        
def poll(client):
    global connected
    isconnected = station.isconnected()
    #print('wifi connected', isconnected)
    if connected == isconnected: # no change
        return
    connected = isconnected
    if not connected:
        addrs = self.station.ifconfig()
        print('wifi connection success', addrs)

        host = addrs[3]
        if client.host != host:
            print('wifi connecting to pypilot at', host)
        client.host = host
        client.disconnect()
    else: # disconnected
        print('wifi disconnected')
        client.disconnect()
