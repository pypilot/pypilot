import Adafruit_Nokia_LCD as LCD
import Adafruit_GPIO.SPI as SPI

from PIL import Image


# Raspberry Pi hardware SPI config:
DC = 23
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
        super(screen, self).__init__(44, 84, 1, None)
        # Hardware SPI usage:
        disp = LCD.PCD8544(DC, RST, spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE, max_speed_hz=4000000))
        # Software SPI usage (defaults to bit-bang SPI interface):
        #disp = LCD.PCD8544(DC, RST, SCLK, DIN, CS)

        # Initialize library.
        disp.begin(contrast=60)

        # Clear display.
        disp.clear()
        disp.display()
        self.disp = disp

    def refresh(self):
        for row in range(6):
            # Iterate through all 83 x axis columns.
            for x in range(84):
                # Set the bits for the column of pixels at the current position.
                bits = 0
                # Don't use range here as it's a bit slow
                for bit in [0, 1, 2, 3, 4, 5, 6, 7]:
                    bits = bits << 1
                    bits |= 1 if self.getpixel(row*ROWPIXELS+7-bit, x) == 0 else 0
                    # Update buffer byte and increment to next byte.
                    self.disp._buffer[index] = bits
                index += 1
            
        disp.display()
