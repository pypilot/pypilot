import pylirc as LIRC
import time
LIRC.init('pypilot')

while True:
    code = LIRC.nextcode(1)
    if code:
        print(code)
    time.sleep(.1)

