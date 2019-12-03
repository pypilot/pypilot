import time, spidev

spi = spidev.SpiDev()
spi.open(0, 1)
spi.max_speed_hz=5000

i = 0
while True:
    x = spi.xfer([0, 0, 0, 0])
    i += 1
    print(i, x)
    time.sleep(.1)
