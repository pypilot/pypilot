if False: # reset on failure
    try:
        import lcd_esp32
    except:
        import machine
        machine.reset()

# get backtrace for debugging
import lcd_esp32

