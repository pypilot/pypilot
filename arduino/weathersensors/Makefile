#
# arduino makefile
#
# build: make
# upload: make upload
# serial monitor: make serial

SKETCHNAME = weathersensors
#DEVICE = /dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0
DEVICE = /dev/ttyUSB1

MCU = atmega328p

ARDUINO_PATH = /usr/share/arduino
CC = $(ARDUINO_PATH)/hardware/tools/avr/bin/avr-gcc
CXX = $(ARDUINO_PATH)/hardware/tools/avr/bin/avr-g++
OBJCOPY = $(ARDUINO_PATH)/hardware/tools/avr/bin/avr-objcopy


CXXFLAGS = -g -Os -Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=$(MCU) -DF_CPU=16000000L -DUSB_VID=null -DUSB_PID=null -DARDUINO=105 -D__PROG_TYPES_COMPAT__ -I$(ARDUINO_PATH)/hardware/arduino/cores/arduino -I$(ARDUINO_PATH)/hardware/arduino/variants/standard  -I$(ARDUINO_PATH)/libraries/Wire -I$(ARDUINO_PATH)/libraries/Wire/utility  -I$(ARDUINO_PATH)/libraries/SPI -I.
LDFLAGS = -Os -Wl,--gc-sections -lm


ARDUINO_SRCS = \
$(ARDUINO_PATH)/libraries/Wire/Wire.cpp \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/avr-libc/realloc.c \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/avr-libc/malloc.c \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/wiring_digital.c \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/wiring.c \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/wiring_analog.c \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/wiring_shift.c \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/wiring_pulse.c \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/WInterrupts.c \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/new.cpp \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/WString.cpp \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/Stream.cpp \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/Print.cpp \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/HardwareSerial.cpp \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/WMath.cpp \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/main.cpp \
/usr/share/arduino/libraries/SPI/SPI.cpp \

#$(ARDUINO_PATH)/libraries/Wire/utility/twi.c \

SRCS = $(ARDUINO_SRCS)

OBJSC = $(SRCS:.cpp=.o)
OBJSD = $(OBJSC:.c=.o)


$(SKETCHNAME).cpp.hex: $(SKETCHNAME).cpp.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

$(SKETCHNAME).cpp.eep: $(SKETCHNAME).cpp.elf
	$(OBJCOPY) -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $< $@

$(SKETCHNAME).cpp.elf: $(OBJSD) obj/$(SKETCHNAME).o objs obj/twi.o obj/PCD8544.o
	$(CC) -mmcu=$(MCU) $(LDFLAGS) -o $@ $(OBJS)

.PHONY: objs
objs:
	echo $(OBJSD) | sed s,$(ARDUINO_PATH),obj$(ARDUINO_PATH),g > objs
        OBJS = $(shell cat objs) obj/$(SKETCHNAME).o obj/twi.o obj/PCD8544.o

obj/$(SKETCHNAME).o : $(SKETCHNAME).ino
	cp $< obj/$<.cpp
	$(CXX) $(CXXFLAGS) -c obj/$<.cpp -o $@

AVRDUDE = avrdude
BOOTLOADER = arduino
BAUDRATE = 57600
upload: $(SKETCHNAME).cpp.hex
	$(AVRDUDE) -P $(DEVICE) -u -p $(MCU) -c $(BOOTLOADER) -U f:w:$(SKETCHNAME).cpp.hex -b $(BAUDRATE)

serial:
	stty -F $(DEVICE) ignbrk -icrnl -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke min 1 time 5 38400
	cat $(DEVICE)

obj/twi.o : twi.c
	$(CC) $(CXXFLAGS) -c $< -o $@

obj/PCD8544.o : PCD8544.cpp charset.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o : %.cpp
	mkdir -p obj$(@:.o=)
	$(CXX) $(CXXFLAGS) -c $< -o obj$@

%.o : %.c
	mkdir -p obj$(@:.o=)
	$(CC) $(CXXFLAGS) -c $< -o obj$@


charset.h: make_charset.py font.ttf
	python make_charset.py > charset.h
