#
# arduino makefile
#
# build: make
# upload: make upload
# serial monitor: make serial

SKETCHNAME = motor
DEVICE = /dev/ttyUSB0
#DEVICE = /dev/ttyAMA0

MCU = atmega328p

ARDUINO_PATH = /usr/share/arduino
CC = $(ARDUINO_PATH)/hardware/tools/avr/bin/avr-gcc
CXX = $(ARDUINO_PATH)/hardware/tools/avr/bin/avr-g++
OBJCOPY = $(ARDUINO_PATH)/hardware/tools/avr/bin/avr-objcopy


CXXFLAGS = -g -Os -Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=$(MCU) -DF_CPU=16000000L -DUSB_VID=null -DUSB_PID=null -DARDUINO=105 -D__PROG_TYPES_COMPAT__ -I$(ARDUINO_PATH)/hardware/arduino/cores/arduino -I$(ARDUINO_PATH)/hardware/arduino/variants/standard -I$(ARDUINO_PATH)/libraries/EEPROM
LDFLAGS = -Os -Wl,--gc-sections -lm


ARDUINO_SRCS = \
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
$(ARDUINO_PATH)/libraries/EEPROM/EEPROM.cpp

SRCS = $(ARDUINO_SRCS)

OBJSC = $(SRCS:.cpp=.o)
OBJSD = $(OBJSC:.c=.o)


$(SKETCHNAME).cpp.hex: $(SKETCHNAME).cpp.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

$(SKETCHNAME).cpp.eep: $(SKETCHNAME).cpp.elf
	$(OBJCOPY) -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $< $@

$(SKETCHNAME).cpp.elf: $(OBJSD) obj/$(SKETCHNAME).o objs
	$(CC) -mmcu=$(MCU) $(LDFLAGS) -o $@ $(OBJS)

.PHONY: objs
objs:
	echo $(OBJSD) | sed s,$(ARDUINO_PATH),obj$(ARDUINO_PATH),g > objs
        OBJS = $(shell cat objs) obj/$(SKETCHNAME).o

obj/$(SKETCHNAME).o : $(SKETCHNAME).ino
	cp $< obj/$<.cpp
	$(CXX) $(CXXFLAGS) -c obj/$<.cpp -o $@ -I.

AVRDUDE = avrdude
BOOTLOADER = arduino
BAUDRATE = 57600
upload: $(SKETCHNAME).cpp.hex
	#	./jump_bootloader
	$(AVRDUDE) -P $(DEVICE) -u -p $(MCU) -c $(BOOTLOADER) -U f:w:$(SKETCHNAME).cpp.hex -b $(BAUDRATE)

serial:
	stty -F $(DEVICE) ignbrk -icrnl -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke min 1 time 5 38400
	cat $(DEVICE)

%.o : %.cpp
	mkdir -p obj$(@:.o=)
	$(CXX) $(CXXFLAGS) -c $< -o obj$@

%.o : %.c
	mkdir -p obj$(@:.o=)
	$(CC) $(CXXFLAGS) -c $< -o obj$@

clean:
	rm -r obj objs $(SKETCHNAME).cpp.elf $(SKETCHNAME).cpp.hex



