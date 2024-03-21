#
# arduino makefile
#
# build: make
# upload: make upload
# serial monitor: make serial

SKETCHNAME = rfremote
DEVICE = /dev/ttyUSB0
MCU = atmega328p

ARDUINO_PATH = /usr/share/arduino
CC = $(ARDUINO_PATH)/hardware/tools/avr/bin/avr-gcc
CXX = $(ARDUINO_PATH)/hardware/tools/avr/bin/avr-g++
OBJCOPY = $(ARDUINO_PATH)/hardware/tools/avr/bin/avr-objcopy

CXXFLAGS = -g -Os -Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=$(MCU) -DF_CPU=2000000L -DUSB_VID=null -DUSB_PID=null -DARDUINO=105 -D__PROG_TYPES_COMPAT__ -I$(ARDUINO_PATH)/hardware/arduino/cores/arduino -I$(ARDUINO_PATH)/hardware/arduino/variants/standard  -I$(ARDUINO_PATH)/libraries/Wire -I$(ARDUINO_PATH)/libraries/Wire/utility  -I$(ARDUINO_PATH)/libraries/SPI -I. -I../hat
LDFLAGS = -Os -Wl,--gc-sections -lm

ARDUINO_SRCS = \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/wiring_digital.c \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/WInterrupts.c \
$(ARDUINO_PATH)/hardware/arduino/cores/arduino/main.cpp \

SRCS = $(ARDUINO_SRCS) RCSwitch.cpp wiring.c

OBJSC = $(SRCS:.cpp=.o)
OBJSD = $(OBJSC:.c=.o)

$(SKETCHNAME).cpp.hex: $(SKETCHNAME).cpp.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

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
BOOTLOADER = stk500v1
BAUDRATE = 19200

#stty -F /dev/ttyUSB0 -icrnl -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke min 0 time 0
#stty -F /dev/ttyUSB0 crtscts
prog: $(SKETCHNAME).cpp.hex
	-$(AVRDUDE) -C/usr/share/arduino/hardware/tools/avrdude.conf -v -p$(MCU) -cstk500v1 -P$(DEVICE) -b19200 -e
	$(AVRDUDE) -P $(DEVICE) -u -p $(MCU) -c $(BOOTLOADER) -b $(BAUDRATE)  -Uefuse:w:0xFE:m -Uhfuse:w:0xDB:m -Ulfuse:w:0x7F:m
	$(AVRDUDE) -C/usr/share/arduino/hardware/tools/avrdude.conf -v -patmega328p -c $(BOOTLOADER) -P $(DEVICE) -b $(BAUDRATE) -Uflash:w:$(SKETCHNAME).cpp.hex:i -Ulock:w:0xc0:m


clean:
	rm -rf obj objs

%.o : %.cpp
	mkdir -p obj/$(@:.o=)
	$(CXX) $(CXXFLAGS) -c $< -o obj/$@

%.o : %.c
	mkdir -p obj/$(@:.o=)
	$(CC) $(CXXFLAGS) -c $< -o obj/$@
