#
# arduino makefile
#
# build: make
# upload: make upload
# serial monitor: make serial

SKETCHNAME = hat
#DEVICE = /dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0
#DEVICE = /dev/ttyAMA0
DEVICE = /dev/spidev0.1

#MCU = atmega88p
MCU = atmega328p

ARDUINO_PATH = /usr/share/arduino
CC = $(ARDUINO_PATH)/hardware/tools/avr/bin/avr-gcc
CXX = $(ARDUINO_PATH)/hardware/tools/avr/bin/avr-g++
OBJCOPY = $(ARDUINO_PATH)/hardware/tools/avr/bin/avr-objcopy

CXXFLAGS = -g -Os -Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=$(MCU) -DF_CPU=8000000L -DUSB_VID=null -DUSB_PID=null -DARDUINO=105 -D__PROG_TYPES_COMPAT__ -I$(ARDUINO_PATH)/hardware/arduino/cores/arduino -I$(ARDUINO_PATH)/hardware/arduino/variants/standard  -I$(ARDUINO_PATH)/libraries/Wire -I$(ARDUINO_PATH)/libraries/Wire/utility  -I$(ARDUINO_PATH)/libraries/SPI -I. -IIRLib2/IRLib2 -IIRLib2/IRLibProtocols -IIRLib2/IRLibRecvPCI -IIRLib2/IRLibRecv
LDFLAGS = -Os -Wl,--gc-sections -lm

ARDUINO_SRCS = \
$(ARDUINO_PATH)/libraries/Wire/Wire.cpp \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/wiring_digital.c \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/wiring_analog.c \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/WInterrupts.c \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/new.cpp \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/WString.cpp \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/Stream.cpp \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/Print.cpp \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/WMath.cpp \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/main.cpp \
$(ARDUINO_PATH)/libraries/SPI/SPI.cpp \
$(ARDUINO_PATH)/libraries/Wire/utility/twi.c \

IRLib2_SRCS = \
IRLib2/IRLibRecvPCI/IRLibRecvPCI.cpp \
IRLib2/IRLib2/IRLibRecvBase.cpp \
IRLib2/IRLib2/IRLibDecodeBase.cpp \
IRLib2/IRLibProtocols/IRLibHardware.cpp \

SRCS = $(ARDUINO_SRCS) $(IRLib2_SRCS) RCSwitch.cpp wiring.c

OBJSC = $(SRCS:.cpp=.o)
OBJSD = $(OBJSC:.c=.o)

$(SKETCHNAME).cpp.hex: $(SKETCHNAME).cpp.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

$(SKETCHNAME).cpp.eep: $(SKETCHNAME).cpp.elf
	$(OBJCOPY) -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $< $@

objs: Makefile
	echo $(OBJSD) | sed s,^,obj/,g | sed 's, , obj/,g' | sed s,//,/,g > objs
	make $(OBJSD)

OBJS = $(shell cat objs) obj/$(SKETCHNAME).o
$(SKETCHNAME).cpp.elf: objs $(OBJS) obj/$(SKETCHNAME).o
	$(CC) -mmcu=$(MCU) $(LDFLAGS) -o $@ $(OBJS)

obj/$(SKETCHNAME).o : $(SKETCHNAME).ino
	cp $< obj/$<.cpp
	$(CXX) $(CXXFLAGS) -c obj/$<.cpp -o $@

AVRDUDE = sudo avrdude
BOOTLOADER = linuxspi
BAUDRATE = 1000000
#sudo avrdude -P /dev/spidev0.1 -u -p atmega328p -c linuxspi -U f:w:hat.cpp.hex -b 1000000
upload: $(SKETCHNAME).cpp.hex
	echo 16 > /sys/class/gpio/export || echo ok
	echo in > /sys/class/gpio/gpio16/direction
	echo out > /sys/class/gpio/gpio16/direction
	echo 1 > /sys/class/gpio/gpio16/value
	sleep .01
	echo 0 > /sys/class/gpio/gpio16/value
	$(AVRDUDE) -P $(DEVICE) -u -p $(MCU) -c $(BOOTLOADER) -U f:w:$(SKETCHNAME).cpp.hex -b $(BAUDRATE)
	echo in > /sys/class/gpio/gpio16/direction

download:
	echo 16 > /sys/class/gpio/export || echo ok
	echo out > /sys/class/gpio/gpio16/direction
	$(AVRDUDE) -P $(DEVICE) -u -p $(MCU) -c $(BOOTLOADER) -U f:r:download.hex -b $(BAUDRATE)
	echo in > /sys/class/gpio/gpio16/direction

reset:
	echo 16 > /sys/class/gpio/export || echo ok
	echo out > /sys/class/gpio/gpio16/direction
	echo in > /sys/class/gpio/gpio16/direction

install: $(SKETCHNAME).cpp.hex
	cp $(SKETCHNAME).cpp.hex ~/.pypilot/$(SKETCHNAME).hex

setfuses:
	echo 16 > /sys/class/gpio/export || echo ok
	echo in > /sys/class/gpio/gpio16/direction
	echo out > /sys/class/gpio/gpio16/direction
	$(AVRDUDE) -P $(DEVICE) -u -p $(MCU) -c $(BOOTLOADER) -b 10000  -Uefuse:w:0xfd:m -Uhfuse:w:0xDA:m -Ulfuse:w:0xFF:m -Ulock:w:0xff:m
	echo in > /sys/class/gpio/gpio16/direction

serial:
	stty -F $(DEVICE) ignbrk -icrnl -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke min 1 time 5 38400
	cat $(DEVICE)

clean:
	rm -rf obj objs

%.o : %.cpp
	mkdir -p obj/$(@:.o=)
	$(CXX) $(CXXFLAGS) -c $< -o obj/$@

%.o : %.c
	mkdir -p obj/$(@:.o=)
	$(CC) $(CXXFLAGS) -c $< -o obj/$@
