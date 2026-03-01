import time

import pylirc as LIRC

LIRC.init('pypilot')

while True:
    code = LIRC.nextcode(1)
    if code:
        print(code)
    time.sleep(.1)

