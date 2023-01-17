#
# arduino makefile
#
# build: make
# upload: make upload
# serial monitor: make serial

SKETCHNAME = motor
DEVICE = /dev/ttyUSB0
#DEVICE = /dev/ttyAMA0

#MCU = atmega328pb
MCU = atmega328p

#ARDUINO_PATH = /usr/share/arduino
ARDUINO_PATH = /opt/arduino-1.8.19

#AVR_TOOLS = /usr/bin
AVR_TOOLS = $(ARDUINO_PATH)/hardware/tools/avr/bin

CC = $(AVR_TOOLS)/avr-gcc
CXX = $(AVR_TOOLS)/avr-g++
OBJCOPY = $(AVR_TOOLS)/avr-objcopy

CXXFLAGS = -g -Os -Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=$(MCU) -DF_CPU=16000000L -DUSB_VID=null -DUSB_PID=null -DARDUINO=105 -D__PROG_TYPES_COMPAT__ -I$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino -I$(ARDUINO_PATH)/hardware/arduino/avr/variants/standard -I$(ARDUINO_PATH)/hardware/arduino/avr/libraries/EEPROM/src
LDFLAGS = -Os -Wl,--gc-sections -lm


ARDUINO_SRCS = \
$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino/wiring_digital.c \
$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino/wiring.c \
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
$(ARDUINO_PATH)/hardware/arduino/avr/cores/arduino/main.cpp

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
#AVRDUDE = $(ARDUINO_PATH)/hardware/tools/avr/bin/avrdude
#AVRDUDE_CONF = $(ARDUINO_PATH)/hardware/tools/avrdude.conf
AVRDUDE_CONF = /etc/avrdude.conf
BOOTLOADER = arduino
BAUDRATE = 57600
BOOTLOADER_HEX = $(ARDUINO_PATH)/hardware/arduino/avr/bootloaders/atmega/ATmegaBOOT_168_atmega328_pro_8MHz.hex

#BOOTLOADER = stk500v1
#BAUDRATE = 19200

upload: $(SKETCHNAME).cpp.hex
	#	./jump_bootloader
	$(AVRDUDE) -P $(DEVICE) -u -p $(MCU) -c $(BOOTLOADER) -U f:w:$(SKETCHNAME).cpp.hex -b $(BAUDRATE)

readfuse:
	stty -F $(DEVICE) -icrnl -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke min 0 time 0
	stty -F $(DEVICE) crtscts
	$(AVRDUDE) -c avrisp -b19200 -P $(DEVICE) -u -p m328p -U efuse:r:-:h
	$(AVRDUDE) -c avrisp -b19200 -P $(DEVICE) -u -p m328p -U lfuse:r:-:h
	$(AVRDUDE) -c avrisp -b19200 -P $(DEVICE) -u -p m328p -U hfuse:r:-:h
	$(AVRDUDE) -c avrisp -b19200 -P $(DEVICE) -u -p m328p -U lock:r:-:h

bootloader:
	stty -F $(DEVICE) -icrnl -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke min 0 time 0
	stty -F $(DEVICE) crtscts
	#sleep 3
	-$(AVRDUDE) -C$(AVRDUDE_CONF) -v -p$(MCU) -cstk500v1 -P$(DEVICE) -b19200 -e
	$(AVRDUDE) -C$(AVRDUDE_CONF) -v -p$(MCU) -cstk500v1 -P$(DEVICE) -b19200 -e -Ulock:w:0xff:m -Uefuse:w:0xfd:m -Uhfuse:w:0xDA:m -Ulfuse:w:0xFF:m
	$(AVRDUDE) -C$(AVRDUDE_CONF) -v -p$(MCU) -cstk500v1 -P$(DEVICE) -b19200 -Uflash:w:$(BOOTLOADER_HEX):i -Ulock:w:0xcf:m

$(SKETCHNAME)_bootloader.hex: $(SKETCHNAME).cpp.hex
	#cat $(SKETCHNAME).cpp.hex $(BOOTLOADER_HEX) > $(SKETCHNAME)_bootloader.hex


program: #$(SKETCHNAME)_bootloader.hex # put both bootloader and program at same time
	stty -F $(DEVICE) 115200
	stty -F $(DEVICE) -brkint -icrnl -imaxbel min 0 time 0
	stty -F $(DEVICE) -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke
	#	./serial
	-$(AVRDUDE) -C$(AVRDUDE_CONF) -v -p$(MCU) -cstk500v1 -P$(DEVICE) -b19200 -e
	#$(AVRDUDE) -C$(AVRDUDE_CONF) -v -p$(MCU) -cstk500v1 -P$(DEVICE) -b19200 -e -Ulock:w:0xff:m -Uefuse:w:0xfd:m -Uhfuse:w:0xDA:m -Ulfuse:w:0xFF:m
	$(AVRDUDE) -C$(AVRDUDE_CONF) -v -p$(MCU) -cstk500v1 -P$(DEVICE) -b19200 -e -Ulock:w:0xff:m -Uefuse:w:0xfd:m -Uhfuse:w:0xDA:m
	$(AVRDUDE) -C$(AVRDUDE_CONF) -v -p$(MCU) -cstk500v1 -P$(DEVICE) -b19200 -Uflash:w:$(SKETCHNAME)_bootloader.hex:i -Ulock:w:0xcf:m -Ulfuse:w:0xFF:m

download:
	-$(AVRDUDE) -C$(AVRDUDE_CONF) -v -p$(MCU) -cstk500v1 -P$(DEVICE) -b19200 -e
	$(AVRDUDE) -C$(AVRDUDE_CONF) -v -p$(MCU) -cstk500v1 -P$(DEVICE) -b19200 -Uflash:r:downloaded.hex:i


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



serial: serial.c
	gcc -o serial serial.c
