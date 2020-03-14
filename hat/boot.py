if False: # reset on failure
    try:
        import lcd_esp32
    except:
        import machine
        machine.reset()

from config_esp32 import config
# get backtrace for debugging
import lcd_esp32
#import testsock
#import upy_client
#upy_client.main()

