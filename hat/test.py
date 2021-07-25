from arduino import arduino
import sys, time

from pypilot.servo import Servo

from pypilot import server, client

def main():
    for i in range(len(sys.argv)):
        if sys.argv[i] == '-t':
            if len(sys.argv) < i + 2:
                print(_('device needed for option') + ' -t')
                exit(1)
            test(sys.argv[i+1])
    
    print('pypilot Servo')
    from server import pypilotServer
    server = pypilotServer()

    from client import pypilotClient
    client = pypilotClient(server)

    from sensors import Sensors # for rudder feedback
    sensors = Sensors(client)
    servo = Servo(client, sensors)


    print('initializing arduino')
    config = {'host':'localhost','hat':{'arduino':{'device':'/dev/spidev0.1',
                                                   'resetpin':'16'}},
              'arduino.nmea.baud': 4800,
              'arduino.nmea.in': False,
              'arduino.nmea.out': False,
              'arduino.ir': True,
              'arduino.debug': True}

    a = arduino(config)
    dt = 0
    
    period = 0.0
    start = lastt = time.monotonic()
    while True:

        events = a.poll()
        if events:
            print(events)
            for key, count in events:
                if key != 'voltage':
                    #print('go', time.monotonic())
                    if count:
                        servo.command.set(1)
                    else:
                        servo.command.set(0)

        
        servo.poll()
        sensors.poll()
        client.poll()
        server.poll()        

        dt = period - time.monotonic() + lastt
        if dt > 0 and dt < period:
            time.sleep(dt)
            lastt += period
        else:
            lastt = time.monotonic()

if __name__ == '__main__':
    main()
