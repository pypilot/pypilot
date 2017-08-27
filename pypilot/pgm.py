from signalk.server import SignalKServer
from servo import Servo
import time

if __name__ == '__main__':
    import serialprobe
    server = SignalKServer()
    serial_probe = serialprobe.SerialProbe()
    servo = Servo(server, serial_probe)

    for i in range(50):
        servo.send_command()
        servo.poll()
        server.HandleRequests()
        time.sleep(.1)

    for i in range(10):
        servo.driver.reprogram()
        time.sleep(.1)
#    servo.raw_command(0);
