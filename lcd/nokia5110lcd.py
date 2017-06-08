
import ugfx
import Adafruit_Nokia_LCD as LCD
import Adafruit_GPIO.SPI as SPI

from PIL import Image


# Raspberry Pi hardware SPI config:
DC = 25
RST = 24
SPI_PORT = 0
SPI_DEVICE = 0

# Raspberry Pi software SPI config:
# SCLK = 4
# DIN = 17
# DC = 23
# RST = 24
# CS = 8

# Beaglebone Black hardware SPI config:
# DC = 'P9_15'
# RST = 'P9_12'
# SPI_PORT = 1
# SPI_DEVICE = 0

# Beaglebone Black software SPI config:
# DC = 'P9_15'
# RST = 'P9_12'
# SCLK = 'P8_7'
# DIN = 'P8_9'
# CS = 'P8_11'

class screen(ugfx.surface):
    def __init__(self):
        super(screen, self).__init__(48, 84, 1, None)
        # Hardware SPI usage:
        disp = LCD.PCD8544(DC, RST, spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE, max_speed_hz=4000000))
        # Software SPI usage (defaults to bit-bang SPI interface):
        #disp = LCD.PCD8544(DC, RST, SCLK, DIN, CS)

        # Initialize library.
        self.contrast = 60
        self.lastcontrast = False
        disp.begin(contrast=self.contrast)

        # Clear display.
        disp.clear()
        disp.display()
        self.disp = disp

    def refresh(self):
        if self.contrast != self.lastcontrast:
            self.disp.set_contrast(self.contrast)
            self.lastcontrast = self.contrast
        self.disp.command(LCD.PCD8544_SETYADDR)
        self.disp.command(LCD.PCD8544_SETXADDR)
        # Write the buffer.
        self.disp._gpio.set_high(self.disp._dc)
        self.binary_write(self.disp._spi._device.fileno())

    def refreshp(self):
        for col in range(6):
            for y in range(84):
                index = y + (5-col)*84
                bits = 0
                for bit in range(8):
                    bits <<= 1
                    if self.getpixel(col*8+bit, y):
                        bits |= 1
                    self.disp._buffer[index] = bits
                        #self.disp._buffer = self.binary()
        self.disp.display()
