#
# arduino makefile
#
# build: make
# upload: make upload
# serial monitor: make serial

SKETCHNAME = weathersensors
#DEVICE = /dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0
DEVICE = /dev/ttyUSB0

#MCU = atmega328p
MCU = atmega32

ARDUINO_PATH = /usr/share/arduino
CC = $(ARDUINO_PATH)/hardware/tools/avr/bin/avr-gcc
CXX = $(ARDUINO_PATH)/hardware/tools/avr/bin/avr-g++
OBJCOPY = $(ARDUINO_PATH)/hardware/tools/avr/bin/avr-objcopy



ARDUINO_SRCS = \
$(ARDUINO_PATH)/hardware/arduino/avr/libraries/Wire/src/Wire.cpp \
$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino/wiring_digital.c \
$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino/wiring.c \
$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino/hooks.c \
$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino/wiring_analog.c \
$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino/wiring_shift.c \
$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino/wiring_pulse.c \
$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino/WInterrupts.c \
$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino/new.cpp \
$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino/WString.cpp \
$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino/Stream.cpp \
$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino/Print.cpp \
$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino/HardwareSerial.cpp \
$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino/HardwareSerial0.cpp \
$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino/WMath.cpp \
$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino/main.cpp \
$(ARDUINO_PATH)/hardware/arduino/avr/libraries/SPI/src/SPI.cpp

CXXFLAGS = -g -Os -Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=$(MCU) -DF_CPU=16000000L -DARDUINO=1818 -DARDUINO_ARCH_AVR -D__PROG_TYPES_COMPAT__ -I$(ARDUINO_PATH)/hardware/arduino/avr/variants/standard -I$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino -I$(ARDUINO_PATH)/hardware/arduino/avr/libraries/Wire/src/ -I$(ARDUINO_PATH)/hardware/arduino/avr/libraries/Wire/src/utility  -I$(ARDUINO_PATH)/hardware/arduino/avr/libraries/SPI/src/ -I. -std=c++11
LDFLAGS = -Os -Wl,--gc-sections -lm


#$(ARDUINO_PATH)/libraries/Wire/utility/twi.c \

SRCS = $(ARDUINO_SRCS)

OBJSC = $(SRCS:.cpp=.o)
OBJSD = $(OBJSC:.c=.o)


$(SKETCHNAME).cpp.hex: $(SKETCHNAME).cpp.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

$(SKETCHNAME).cpp.eep: $(SKETCHNAME).cpp.elf
	$(OBJCOPY) -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $< $@

$(SKETCHNAME).cpp.elf: $(OBJSD) obj/$(SKETCHNAME).o objs obj/twi.o obj/PCD8544.o obj/JLX12864.o
	$(CC) -mmcu=$(MCU) $(LDFLAGS) -o $@ $(OBJS)

.PHONY: objs
objs:
	echo $(OBJSD) | sed s,$(ARDUINO_PATH),obj$(ARDUINO_PATH),g > objs
        OBJS = $(shell cat objs) obj/$(SKETCHNAME).o obj/twi.o obj/PCD8544.o obj/JLX12864.o

obj/$(SKETCHNAME).o : $(SKETCHNAME).ino
	cp $< obj/$<.cpp
	$(CXX) $(CXXFLAGS) -c obj/$<.cpp -o $@

AVRDUDE = avrdude
BOOTLOADER = arduino
BAUDRATE = 115200
upload: $(SKETCHNAME).cpp.hex
	$(AVRDUDE) -P $(DEVICE) -u -p $(MCU) -c $(BOOTLOADER) -U f:w:$(SKETCHNAME).cpp.hex -b $(BAUDRATE)

bootloader:
	stty -F /dev/ttyUSB0 -icrnl -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke min 0 time 0
	stty -F /dev/ttyUSB0 crtscts
	#	sleep 3
	$(AVRDUDE) -C/usr/share/arduino/hardware/tools/avrdude.conf -v -patmega328p -cstk500v1 -P/dev/ttyUSB0 -b19200 -e
	$(AVRDUDE) -C/usr/share/arduino/hardware/tools/avrdude.conf -v -patmega328p -cstk500v1 -P/dev/ttyUSB0 -b19200 -e -Ulock:w:0xff:m -Uefuse:w:0xfd:m -Uhfuse:w:0xDA:m -Ulfuse:w:0xFF:m 
	$(AVRDUDE) -C/usr/share/arduino/hardware/tools/avrdude.conf -v -patmega328p -cstk500v1 -P/dev/ttyUSB0 -b19200 -Uflash:w:/usr/share/arduino/hardware/arduino/bootloaders/atmega/ATmegaBOOT_168_atmega328.hex:i -Ulock:w:0xcf:m


serial:
	stty -F $(DEVICE) ignbrk -icrnl -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke min 1 time 5 38400
	cat $(DEVICE)

obj/twi.o : twi.c
	$(CC) $(CXXFLAGS) -c $< -o $@

obj/PCD8544.o : PCD8544.cpp charset.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

obj/JLX12864.o : JLX12864.cpp charset.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o : %.cpp
	mkdir -p obj$(@:.o=)
	$(CXX) $(CXXFLAGS) -c $< -o obj$@

%.o : %.c
	mkdir -p obj$(@:.o=)
	$(CC) $(CXXFLAGS) -c $< -o obj$@


charset.h: make_charset.py font.ttf
	python3 make_charset.py > charset.h

clean:
	rm -rf objs obj
