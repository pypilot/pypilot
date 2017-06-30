
import ugfx
import Adafruit_Nokia_LCD as LCD
import Adafruit_GPIO.SPI as SPI

from PIL import Image


# Raspberry Pi hardware SPI config:
DC = 22
#DC = 25
RST = 18
#RST = 24
SPI_PORT = 1
SPI_DEVICE = 0

# Raspberry Pi software SPI config:
SCLK = 23
DIN = 19
# DC = 23
# RST = 24
CS = 24

class screen(ugfx.surface):
    def __init__(self):
        super(screen, self).__init__(48, 84, 1, None)
        # Hardware SPI usage:
        disp = LCD.PCD8544(DC, RST, spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE, max_speed_hz=4000000))
        self.refresh = self.refreshhw

        # Software SPI usage (defaults to bit-bang SPI interface):
 #       disp = LCD.PCD8544(DC, RST, SCLK, DIN, CS)
#        self.refresh = self.refreshsw

        # Initialize library.
        self.contrast = 40
        self.lastcontrast = False
        disp.begin(contrast=self.contrast)

        # Clear display.
        disp.clear()
        disp.display()
        self.disp = disp


    def refreshhw(self):
        if self.contrast != self.lastcontrast:
            self.disp.set_contrast(self.contrast)
            self.lastcontrast = self.contrast
        self.disp.command(LCD.PCD8544_SETYADDR)
        self.disp.command(LCD.PCD8544_SETXADDR)

        # Write the buffer.
	self.disp.set_bias(4)
        self.disp.set_contrast(self.contrast)
        self.disp._gpio.set_high(self.disp._dc)
        self.binary_write(self.disp._spi._device.fileno())

    def refreshsw(self):
	self.disp.set_bias(4)
        self.disp.set_contrast(self.contrast)
        self.disp.command(LCD.PCD8544_SETYADDR)
        self.disp.command(LCD.PCD8544_SETXADDR)

        self.disp._gpio.set_high(self.disp._dc)
        self.disp._gpio.set_low(self.disp._spi._ss)

        if True:
            self.binary_write_sw(self.disp._spi._sclk, self.disp._spi._mosi)
        else:
            for col in range(6):
                for y in range(84):
                    for bit in range(8):
                        # Write bit to MOSI.
                        if self.getpixel((5-col)*8+bit, y):
                            self.disp._gpio.set_high(self.disp._spi._mosi)
                        else:
                            self.disp._gpio.set_low(self.disp._spi._mosi)
                        # Flip clock off base.
                        self.disp._gpio.output(self.disp._spi._sclk, not self.disp._spi._clock_base)
                        # Return clock to base.
                        self.disp._gpio.output(self.disp._spi._sclk, self.disp._spi._clock_base)

        self.disp._gpio.set_high(self.disp._spi._ss)
        
    def refreshold(self):
        for col in range(6):
            for y in range(84):
                index = y + (5-col)*84
                bits = 0
                for bit in range(8):
                    bits <<= 1
                    if self.getpixel(col*8+bit, y):
                        bits |= 1

                    self.disp._buffer[index] = bits

        # set bias and contrast every frame allows hot-plugging
	self.disp.set_bias(4)
        self.disp.set_contrast(self.contrast)
        self.disp.display()
