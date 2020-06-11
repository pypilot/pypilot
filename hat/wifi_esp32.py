import network

#essid, psk = 'openplotter', '12345678'
essid, psk = 'pypilot', None

station = network.WLAN(network.STA_IF)  # client, not AP
station.active(True) # enable wifi
if psk:
    station.connect(essid, psk)
else:
    station.connect(essid)
connected = False
        
def poll(client):
    global connected
    isconnected = station.isconnected()
    if connected == isconnected: # no change
        return connected
    connected = isconnected
    if connected:
        addrs = station.ifconfig()
        print('wifi connection success', addrs)

        host = addrs[3]
        if client.host != host:
            print('wifi connecting to pypilot at', host)
        client.host = host
    else: # disconnected
        print('wifi disconnected')
        client.disconnect()
    return connected
