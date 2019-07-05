/* Copyright (C) 2019 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

#include <Arduino.h>
#include <stdint.h>
#include <HardwareSerial.h>

#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/boot.h>
#include <util/delay.h>

/*
This program is meant to interface with pwm based
motor controller either brushless or brushed, or a regular RC servo

You may need to modify the source code to support different hardware

adc pin0 is a resistor divider to measure voltage
             allowing up to 20 volts (10k and 560 ohm, 1.1 volt reference)
adc pin1 goes to .01/.05 ohm shunt to measure current
adc pin2 goes to 100k resistor to 5v and 10k NTC thermistor to gnd ctrl temp
adc pin3 goes to 100k resistor to 5v and 10k NTC thermistor to gnd motor temp
adc pin4 rudder sense
unused analog pins should be grounded

digital pins 4 and 5 determine the current sense as folows:
pin 4 determines range
pin 5 determines high/low current  (20A or 60A max)

D4  D5
 1   1        .05 ohm, (or .001 ohm x 50 gain)
 0   1        .01 ohm
 1   0        .0005 ohm x 50 gain
 0   0        .0005 ohm x 200 gain   *ratiometric mode


Ratiometric Mode:
for D4=0 and D5=0, the adc operates over the 0-5 volt range
making it ratiometric (linearly accurate) for rudder feedback
and reduces impedance in the rudder measurement
the temperature resistors are changed to 10k and 10k ntc
voltage measurement accuracy is reduced, and the resistors used are
1k5 and 10k for a range of 38 volts.   R12 is not used in this mode.


digital pin6 determines RC pwm if 1, or Hbridge if 0.
if RC pwm:
   digital pin9 pwm output standard ESC (1-2 ms pulse every 20 ms)
           pin2 esc programming input/output (with arduinousblinker script)
if Hbridge
   digital pin2 and pin3 for low side, pin9 and pin10 for high side


optional:
digital pin7 forward fault for optional switch to stop forward travel
digital pin8 reverse fault for optional switch to stop reverse travel

If Pin 12 has 560 ohm resistor to A0, then 24 volts is supported,
this allows for measuring voltage up to 40.4 volts, without losing
resolution if operating below 20 volts

D12
 1    0-20.75 volts (560 and 10k resistor)  resolution 0.02 volts
 0    0-40.4  volts (280 and 10k resistor)  resolution 0.04 volts

digital pin13 is led on when engaged

The program uses a simple protocol to ensure only
correct data can be received and to ensure that
false/incorrect or random data is very unlikely to
produce motor movement.

The input and output over uart has 4 byte packets

The first byte is the command or register, the next
two bytes is a 16bit value (signed or unsigned)
the last byte is a crc8 of the first 3 bytes

If incoming data has the correct crc for a few frames
the command can be recognized.

*/

/* vnh2sp30 is supported, but warning, I received 3 boards:
-  reverse is half power making chip very hot
-  reverse does not work
-  current sense does not work

So in some way, 3 out of 3 were defective and I cannot recommend
this part at all unfortunately.

vnh2sp30  <->  arduino <->  CPC5001
+5V              5v
GND              GND
EN               D10
CS               A1
INA              D2
INB              D3
PWM              D9

If used with optical isolation (strongly recommended)
PWR+             VIN
                 5v         vcc
                 tx         rx
                 rx         tx
                 gnd        gnd
*/

//#define VNH2SP30 // defined if this board is used
//#define DISABLE_TEMP_SENSE    // if no temp sensors avoid errors
//#define DISABLE_VOLTAGE_SENSE // if no voltage sense
//#define DISABLE_RUDDER_SENSE  // if no rudder sense


// run at 4mhz instead of 16mhz to save power,
// and to be able to measure lower current from the shunt

#define DIV_CLOCK 4  // 1 for 16mhz, 2 for 8mhz, 4 for 4mhz
#define QUIET  // don't use 1khz

#if DIV_CLOCK==4
#define dead_time \
    asm volatile ("nop"); \
    asm volatile ("nop"); \
    asm volatile ("nop");
#elif DIV_CLOCK==2
#define dead_time \
    asm volatile ("nop"); \
    asm volatile ("nop"); \
    asm volatile ("nop"); \
    asm volatile ("nop"); \
    asm volatile ("nop"); \
    asm volatile ("nop");
#elif DIV_CLOCK==1
#define dead_time \
    asm volatile ("nop"); \
    asm volatile ("nop"); \
    asm volatile ("nop"); \
    asm volatile ("nop"); \
    asm volatile ("nop"); \
    asm volatile ("nop"); \
    asm volatile ("nop"); \
    asm volatile ("nop"); \
    asm volatile ("nop"); \
    asm volatile ("nop"); \
    asm volatile ("nop");
#else
#error "invalid DIV_CLOCK"
#endif

//#define HIGH_CURRENT_OLD   // high current uses 500uohm resistor and 50x amplifier
// otherwise using shunt without amplification

#define shunt_sense_pin 4 // use pin 4 to specify shunt resistance
uint8_t shunt_resistance = 1;

#define low_current_pin 5 // use pin 5 to specify low current (no amplifier)
uint8_t low_current = 1;

#define pwm_style_pin 6
// pwm style, 0 = hbridge, 1 = rc pwm, 2 = vnh2sp30
uint8_t pwm_style = 2; // detected to 0 or 1 unless detection disabled, default 2

#define fwd_fault_pin 7 // use pin 7 for optional fault
#define rev_fault_pin 8 // use pin 7 for optional fault
// if switches pull this pin low, the motor is disengaged
// and will be noticed by the control program

#define pwm_output_pin 9

#define hbridge_a_bottom_pin 2
#define hbridge_b_bottom_pin 3
#define hbridge_a_top_pin 9
#define hbridge_b_top_pin 10
#define enable_pin 10 // for vnh2sp30

// for direct mosfet mode, define how to turn on/off mosfets
// do not use digitalWrite!
#define a_top_on  PORTB |= _BV(PB1)
#define a_top_off PORTB &= ~_BV(PB1)
#define a_bottom_on PORTD |= _BV(PD2)
#define a_bottom_off PORTD &= ~_BV(PD2)
#define b_top_on  PORTB |= _BV(PB2)
#define b_top_off PORTB &= ~_BV(PB2)
#define b_bottom_on PORTD |= _BV(PD3)
#define b_bottom_off PORTD &= ~_BV(PD3)

#define clutch_pin 11 // use pin 11 to engage clutch

#define clutch_on PORTB |= _BV(PB3)
#define clutch_off PORTB &= ~_BV(PB3)

#define voltage_sense_pin 12
uint8_t voltage_sense = 1;
uint8_t voltage_mode = 0;  // 0 = 12 volts, 1 = 24 volts
uint16_t max_voltage = 1800; // 18 volts max by default

#define led_pin 13 // led is on when engaged

#include <stdarg.h>
void debug(char *fmt, ... ){
    char buf[128]; // resulting string limited to 128 chars
    va_list args;
    va_start (args, fmt );
    vsnprintf(buf, 128, fmt, args);
    va_end (args);
    Serial.print(buf);
}

enum commands {COMMAND_CODE=0xc7, RESET_CODE=0xe7, MAX_CURRENT_CODE=0x1e, MAX_CONTROLLER_TEMP_CODE=0xa4, MAX_MOTOR_TEMP_CODE=0x5a, RUDDER_RANGE_CODE=0xb6, REPROGRAM_CODE=0x19, DISENGAGE_CODE=0x68, MAX_SLEW_CODE=0x71, EEPROM_READ_CODE=0x91, EEPROM_WRITE_CODE=0x53};

enum results {CURRENT_CODE=0x1c, VOLTAGE_CODE=0xb3, CONTROLLER_TEMP_CODE=0xf9, MOTOR_TEMP_CODE=0x48, RUDDER_SENSE_CODE=0xa7, FLAGS_CODE=0x8f, EEPROM_VALUE_CODE=0x9a};

enum {SYNC=1, OVERTEMP=2, OVERCURRENT=4, ENGAGED=8, INVALID=16*1, FWD_FAULTPIN=16*2, REV_FAULTPIN=16*4, BADVOLTAGE=16*8, MIN_RUDDER=256*1, MAX_RUDDER=256*2, CURRENT_RANGE=256*4, BAD_FUSES=256*8};

uint16_t flags = 0, faults = 0;
uint8_t serialin, packet_count = 0;

#include <avr/eeprom.h>

// we need these to be atomic for 16 bytes

uint16_t eeprom_read_16(int address)
{
    // ensure atomic update of 16 bits with 3 banks
    uint16_t v[3];
    for(int i=0; i<3; i++) {
        int addr = i*256 + address;
        v[i] = eeprom_read_word((uint16_t*)addr);
        //eeprom_read_byte((unsigned char *)addr) | eeprom_read_byte((unsigned char *)addr+1)<<8;
    }

    if(v[1] == v[2])
        return v[2];

    return v[0];
}

void eeprom_update_16(int address, uint16_t value)
{
    // write in 3 locations
    for(int i=0; i<3; i++) {
        int addr = i*256 + address;
        eeprom_update_word((uint16_t*)addr, value);
//        eeprom_update_byte((unsigned char*)addr, value&0xff);
//        eeprom_update_byte((unsigned char*)addr+1, value>>8);
    }
}

uint8_t eeprom_read_8(int address, uint8_t &value)
{
    static uint8_t lastvalue, lastaddress=255;
    if(address & 1) { // odd
        if(address == lastaddress+1) {
            value = lastvalue;
            return 1;
        } else
            return 0;
    }

    // even
    uint16_t v = eeprom_read_16(address);
    value = v&0xff;
    lastvalue = v >> 8;
    lastaddress = address;
    return 1;
}

void eeprom_update_8(int address, uint8_t value)
{
    static uint8_t lastvalue, lastaddress=255;
    if(address & 1) { // odd
        if(address == lastaddress+1)
            eeprom_update_16(lastaddress, lastvalue | value<<8);
    } else {
        lastvalue = value;
        lastaddress = address;
    }
}


#include "crc.h"

uint16_t max_current = 2000; // 20 Amps
uint16_t max_controller_temp= 7000; // 70C
uint16_t max_motor_temp = 7000; // 70C
uint8_t max_slew_speed = 15, max_slew_slow = 35; // 200 is full power in 1/10th of a second
uint8_t rudder_min = 0, rudder_max = 255;

uint8_t eeprom_read_addr = 0;
uint8_t eeprom_read_end = 0;

#include <avr/pgmspace.h>

void setup()
{
#if DIV_CLOCK==4
    CLKPR = _BV(CLKPCE);
    CLKPR = _BV(CLKPS1); // divide by 4
#elif DIV_CLOCK==2
    CLKPR = _BV(CLKPCE);
    CLKPR = _BV(CLKPS0); // divide by 2
#endif
    // Disable all interrupts
    cli();

/* Clear MCU Status Register. Not really needed here as we don't need to know why the MCU got reset. page 44 of datasheet */
    MCUSR = 0;

/* Disable and clear all Watchdog settings. Nice to get a clean slate when dealing with interrupts */

    WDTCSR = (1<<WDCE)|(1<<WDE);

    // interrupt in 1/4th second
    WDTCSR = (1<<WDIE) | (1<<WDP2);

    // read fuses, and report this as flag if they are wrong
    uint8_t lowBits      = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
    uint8_t highBits     = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
    uint8_t extendedBits = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
    uint8_t lockBits     = boot_lock_fuse_bits_get(GET_LOCK_BITS);
    if(lowBits != 0xFF || highBits != 0xda ||
       (extendedBits != 0xFD && extendedBits != 0xFC)
       // || lockBits != 0xCF // too many clones don't set lock bits and there is no spm
        )
        flags |= BAD_FUSES;

    sei(); // Enable all interrupts.

    // enable pullup on unused pins (saves .5 mA)
    pinMode(A4, INPUT);
    digitalWrite(A4, LOW);

    pinMode(shunt_sense_pin, INPUT_PULLUP);
    pinMode(low_current_pin, INPUT_PULLUP);
    pinMode(pwm_style_pin, INPUT_PULLUP);
    pinMode(clutch_pin, INPUT_PULLUP);
    pinMode(voltage_sense_pin, INPUT_PULLUP);

    serialin = 0;
    // set up Serial library
    Serial.begin(38400*DIV_CLOCK);

    set_sleep_mode(SLEEP_MODE_IDLE); // wait for serial

    digitalWrite(A0, LOW);
    pinMode(A0, OUTPUT);
    voltage_sense = digitalRead(voltage_sense_pin);
    if(!voltage_sense)
        pinMode(voltage_sense_pin, INPUT); // if attached, turn off pullup
    pinMode(A0, INPUT);

    digitalWrite(clutch_pin, LOW);
    pinMode(clutch_pin, OUTPUT); // clutch

    digitalWrite(led_pin, LOW);
    pinMode(led_pin, OUTPUT); // status LED

    pinMode(fwd_fault_pin, INPUT);
    digitalWrite(fwd_fault_pin, HIGH); /* enable internal pullups */
    pinMode(rev_fault_pin, INPUT);
    digitalWrite(rev_fault_pin, HIGH); /* enable internal pullups */

    pinMode(pwm_style_pin, INPUT);
    digitalWrite(pwm_style_pin, HIGH); /* enable internal pullups */

    pinMode(shunt_sense_pin, INPUT);
    digitalWrite(shunt_sense_pin, HIGH); /* enable internal pullups */

    pinMode(low_current_pin, INPUT);
    digitalWrite(low_current_pin, HIGH); /* enable internal pullups */

    _delay_us(100); // time to settle

    // test output type, pwm or h-bridge
#ifndef VNH2SP30
    pwm_style = digitalRead(pwm_style_pin);
#endif
    if(pwm_style) {
        digitalWrite(pwm_output_pin, LOW); /* enable internal pullups */
        pinMode(pwm_output_pin, OUTPUT);
    }
    // test shunt type, if pin wired to ground, we have 0.01 ohm, otherwise 0.05 ohm
    shunt_resistance = digitalRead(shunt_sense_pin);

    // test current
    low_current = digitalRead(low_current_pin);

#if 1
    // setup adc
    DIDR0 = 0x3f; // disable all digital io on analog pins
    if(pwm_style == 2)
        ADMUX = _BV(REFS0) | _BV(MUX0); // 5v
    else
        ADMUX = _BV(REFS0)| _BV(REFS1) | _BV(MUX0); // 1.1v
    ADCSRA = _BV(ADEN) | _BV(ADIE); // enable adc with interrupts
#if DIV_CLOCK==4
    ADCSRA |= _BV(ADPS2) | _BV(ADPS1); // divide clock by 64
//    ADCSRA |= _BV(ADPS0) |  _BV(ADPS1) | _BV(ADPS2); // divide clock by 128
#else
    ADCSRA |= _BV(ADPS0) |  _BV(ADPS1) | _BV(ADPS2); // divide clock by 128
#endif
    ADCSRA |= _BV(ADSC); // start conversion
#endif    
}

uint8_t in_bytes[3];
uint8_t sync_b = 0, in_sync_count = 0;

uint8_t out_sync_b = 0, out_sync_pos = 0;
uint8_t crcbytes[3];

uint8_t timeout;
uint8_t rudder_sense = 0;

// command is from 0 to 2000 with 1000 being neutral
uint16_t lastpos = 1000;
void position(uint16_t value)
{
    lastpos = value;
    if(pwm_style == 1)
//        OCR1A = 1200/DIV_CLOCK + value * 6 / 5 / DIV_CLOCK;
        OCR1A = 1500/DIV_CLOCK + value * 3 / 2 / DIV_CLOCK;
    //OCR1A = 1350/DIV_CLOCK + value * 27 / 20 / DIV_CLOCK;
    else if(pwm_style == 2) {
        OCR1A = abs((int)value - 1000) * DIV_CLOCK;
        if(value > 1040) {
            a_bottom_off;
            b_bottom_on;
        } else if(value < 960) {
            b_bottom_off;
            a_bottom_on;
        } else { // low, set pwm for brake
            a_bottom_off;
            b_bottom_off;
        }            
    } else {
        uint16_t OCR1A_u = 16000, OCR1B_u = 16000, ICR1_u = 1000;
        // use 62.5 hz at full power to reduce losses
        // some cycling is required to refresh the bootstrap capacitor
        // but the current through the motor remains continuous
        if(value > 1100) {
#ifndef QUIET                
            ICR1_u=16000/DIV_CLOCK;  //fPWM=1khz
            if(value > 1980)
                value = 1980;
            OCR1A_u = 20 + (2000 - value)*(16/DIV_CLOCK);
#else
            if(value > 1900) {
                if(low_current)
                    ICR1_u=64000;  //fPWM=62.5hz, 125hz, or 250hz
                else
                    ICR1_u=16000;  //faster frequency for high current board
                OCR1A_u = 120; // 99.8125% duty cycle
            } else {
                ICR1_u=1000/DIV_CLOCK;  //fPWM=16khz
                OCR1A_u = 20 + (2000 - value)/DIV_CLOCK;
            }
#endif
            TIMSK1 = _BV(TOIE1) | _BV(OCIE1A);
        } else if(value < 900) {
#ifndef QUIET                
            ICR1_u=16000/DIV_CLOCK;  //fPWM=1khz
            if(value < 20)
                value = 20;
            OCR1B_u = 20 + (value)*16/DIV_CLOCK;
#else
            if(value < 100) {
                if(low_current)
                    ICR1_u=64000;  //fPWM=62.5hz, 125hz, or 250hz
                else
                    ICR1_u=16000;  //faster frequency for high current board
                OCR1B_u = 120;
            } else {
                ICR1_u=1000/DIV_CLOCK;  //fPWM=16khz
                OCR1B_u = 20 + (value)/DIV_CLOCK;
            }
#endif
            TIMSK1 = _BV(TOIE1) | _BV(OCIE1B);
        } else {
            TIMSK1 = 0;//_BV(TOIE1);
            a_top_off;
            b_top_off;
            dead_time;
//            a_bottom_on;  // set brake
//            b_bottom_on;
            a_bottom_off;
            b_bottom_off;
        }

        if(TIMSK1) {
            OCR1A = OCR1A_u;
            OCR1B = OCR1B_u;
            if(ICR1 != ICR1_u) {
                cli();
                ICR1 = ICR1_u;
                TCNT1 = ICR1_u - 40; // ensure timer is before new end
                sei();
            }
        }
    }
}

uint16_t command_value = 1000;
void stop()
{
    position(1000);
    command_value = 1000;
}

void stop_fwd()
{
    if(lastpos > 1000)
       stop();
}

void stop_rev()
{
    if(lastpos < 1000)
       stop();
}

void update_command()
{
    int16_t speed_rate = max_slew_speed;
    // value of 20 is 1 second full range at 50hz
    int16_t slow_rate = max_slew_slow;

    //uint16_t cur_value = OCR1A * 2 / 3 - 1000;
    uint16_t cur_value = lastpos;

    int16_t diff = (int)command_value - (int)cur_value;

    // limit motor speed change to stay within speed and slow slew rates
    if(diff > 0) {
        if(cur_value < 1000) {
            if(diff > slow_rate)
               diff = slow_rate;
        } else
            if(diff > speed_rate)
                diff = speed_rate;
    } else {
        if(cur_value > 1000) {
            if(diff < -slow_rate)
                diff = -slow_rate;
        } else
            if(diff < -speed_rate)
                diff = -speed_rate;
    }

    position(cur_value + diff);
}

void disengage()
{
    stop();
    flags &= ~ENGAGED;
    timeout = 120/DIV_CLOCK+1; // detach in about 62ms
    digitalWrite(clutch_pin, LOW); // clutch
    digitalWrite(led_pin, LOW); // status LED
}

void detach()
{
    if(pwm_style) {
        TCCR1A=0;
        TCCR1B=0;
        while(digitalRead(pwm_output_pin)); // wait for end of pwm if pulse is high
        TIMSK1 = 0;
        if(pwm_style == 2) {
            a_bottom_off;
            b_bottom_off;
            digitalWrite(enable_pin, LOW);
        }
    } else {
        TCCR1A=0;
        TCCR1B=0;
        TIMSK1 = 0;
        a_top_off;
        a_bottom_off;
        b_top_off;
        b_bottom_off;
    }
    TIMSK2 = 0;
    timeout = 128/DIV_CLOCK; // avoid overflow
}

void engage()
{
    if(flags & ENGAGED) {
        //update_command(); // 30hz
        return;
    }

    if(pwm_style == 1) {
        TCNT1 = 0x1fff;
        //Configure TIMER1
        TCCR1A=_BV(COM1A1)|_BV(WGM11);        //NON Inverted PWM
        TCCR1B=_BV(WGM13)|_BV(WGM12)|_BV(CS11); //PRESCALER=8 MODE 14(FAST PWM)
        ICR1=40000/DIV_CLOCK;  //fPWM=50Hz (Period = 20ms Standard).
        TIMSK1 = 0;
    } else if(pwm_style == 2) {
        TCNT1 = 0;
        TCCR1A=_BV(COM1A1)|_BV(WGM11);        //NON Inverted PWM
        TCCR1B=_BV(WGM13)|_BV(WGM12)|_BV(CS10); //PRESCALER=0 MODE 14(FAST PWM)

        // use 1khz safe at all speeds.   20khz is nice to avoid
        // audible sound, but you need to set minimum speed to 20-30%
        // or it will overheat the part at very low speeds.
        ICR1=16000/DIV_CLOCK;  //fPWM=1khz
        TIMSK1 = 0;

        digitalWrite(hbridge_a_bottom_pin, LOW);
        digitalWrite(hbridge_b_bottom_pin, LOW);
        digitalWrite(enable_pin, LOW);

        pinMode(hbridge_a_bottom_pin, OUTPUT);
        pinMode(hbridge_b_bottom_pin, OUTPUT);
        pinMode(enable_pin, INPUT);

        digitalWrite(enable_pin, HIGH);
    } else {
        //Configure TIMER1
        TIMSK1 = 0;
        TCNT1 = 0;
        TCCR1A=_BV(WGM11);        //NON Inverted PWM
        TCCR1B=_BV(WGM13)|_BV(WGM12)|_BV(CS10); //PRESCALER=0 MODE 14(FAST PWM)
        ICR1=16000/DIV_CLOCK;  //fPWM=1khz

        a_top_off;
        a_bottom_off;
        b_top_off;
        b_bottom_off;

        pinMode(hbridge_a_bottom_pin, OUTPUT);
        pinMode(hbridge_b_bottom_pin, OUTPUT);
        pinMode(hbridge_a_top_pin, OUTPUT);
        pinMode(hbridge_b_top_pin, OUTPUT);
    }

    position(1000);
    digitalWrite(clutch_pin, HIGH); // clutch
    digitalWrite(led_pin, HIGH); // status LED

// use timer2 as timeout
    timeout = 0;
    TCNT2 = 0;
    TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20); // divide 1024

    flags |= ENGAGED;
}

// set hardware pwm to period of "(1500 + 1.5*value)/2" or "750 + .75*value" microseconds

enum {CURRENT, VOLTAGE, CONTROLLER_TEMP, MOTOR_TEMP, RUDDER, CHANNEL_COUNT};
const uint8_t muxes[] = {_BV(MUX0), 0, _BV(MUX1), _BV(MUX0) | _BV(MUX1), _BV(MUX2)};

volatile struct adc_results_t {
    uint32_t total;
    uint16_t count;
} adc_results[CHANNEL_COUNT][2];

const uint8_t defmux = _BV(REFS0)| _BV(REFS1); // 1.1v
uint16_t adc_counter;
uint8_t adc_cnt;

// take 4 channels, 4 samples of each using last sample,
// except for current, 50 samples, discarding first 3
// total 62 measurements, each measurement takes 13 cycles,
// channel change takes 12 cycles, total is 854 cycles,
// 16mhz / 128 / 854 = 146 samples/s, except for current,
// has 6862 samples/s
ISR(ADC_vect)
{
    if(pwm_style)
        ADCSRA |= _BV(ADSC); // enable conversion
    else
        sei(); // enable nested interrupts to ensure correct operation

    uint16_t adcw = ADCW;
    if(++adc_cnt <= 3) // discard first few readings after changing channel
        goto ret;

    for(int i=0; i<2; i++) {
        if(adc_results[adc_counter][i].count < 4000) {
            adc_results[adc_counter][i].total += adcw;
            adc_results[adc_counter][i].count++;
        }
    }

    if(adc_counter == CURRENT) { // take more current measurements
        if(adc_cnt < 50)
            goto ret;
    } else if(adc_counter == VOLTAGE) {
        if(adc_cnt < 8)
            goto ret;
    } else if(adc_counter == RUDDER && rudder_sense)
        if(adc_cnt < 16) // take more samples for rudder, if sampled
            goto ret;

    // advance to next channel
    adc_cnt = 0;
    
    if(++adc_counter >= CHANNEL_COUNT)
        adc_counter=0;
#ifdef DISABLE_VOLTAGE_SENSE
    if(adc_counter == VOLTAGE)
        adc_counter++;
#endif
#ifdef DISABLE_TEMP_SENSE
    if(adc_counter == CONTROLLER_TEMP)
        adc_counter+=2;
#endif
#ifdef DISABLE_RUDDER
    if(adc_counter == RUDDER)
        adc_counter=0;
#endif
    ADMUX = defmux | muxes[adc_counter];
ret:;
    if(!pwm_style)
        ADCSRA |= _BV(ADSC); // enable conversion
}

uint16_t CountADC(uint8_t index, uint8_t p)
{
#ifdef DISABLE_VOLTAGE_SENSE
    if(index == VOLTAGE)
        return 0;
#endif
#ifdef DISABLE_TEMP_SENSE
    if(index == CONTROLLER_TEMP || index == MOTOR_TEMP)
        return 0;
#endif
#ifdef DISABLE_RUDDER_SENSE
    if(index == RUDDER)
        return 0;
#endif
    return adc_results[index][p].count;
}

// return value from 0-16368
uint16_t TakeADC(uint8_t index, uint8_t p)
{
    uint32_t t, c;
    uint8_t lp = 0; // don't lowpass
    cli();
    t = adc_results[index][p].total;
    c = adc_results[index][p].count;
    if(lp) { // fault measurements are not lowpassed
        adc_results[index][p].total >>= 1;
        adc_results[index][p].count >>= 1;
    } else {
        adc_results[index][p].total = 0;
        adc_results[index][p].count = 0;
    }

    sei();
    if(c == 0)
        return 0;

    uint32_t avg = 16 * t / c;
    if(lp && c&1) { // correct rounding errors from lowapss
        int havg = avg>>5;
        cli();
        adc_results[index][p].total -= havg;
        sei();
    }
    return avg; // multiply by 16 to keep less significant bits
}

uint16_t TakeAmps(uint8_t p)
{
    uint32_t v = TakeADC(CURRENT, p);

    if(pwm_style == 2) // VNH2SP30
        return v * 9 / 34 / 16;
    
    if(low_current) {
    // current units of 10mA
    // 275 / 128 = 100.0/1024/.05*1.1   for 0.05 ohm shunt
    // 1375 / 128 = 100.0/1024/.01*1.1   for 0.01 ohm shunt
        if(shunt_resistance)
            v = v * 275 / 128 / 16;

        if(v > 16)
            v = v * 1375 / 128 / 16 + 16; // 550mA minimum
        else
            v = 0;
    } else { // high current
        // high curront controller has .0005 ohm with 50x gain
        // 275/64 = 100.0/1024/.0005/50*1.1
#if defined(HIGH_CURRENT_OLD)
        // high curront controller has .001 ohm with 50x gain
        // 275/128 = 100.0/1024/.001/50*1.1
        v = v * 275 / 128 / 16;
#else
        if(v > 16)
            v = v * 275 / 64 / 16; // 820mA offset
        else
            v = 0;
#endif        
    }
    if(v == 0)
        return 0;

    return v;
}

uint16_t TakeVolts(uint8_t p)
{
    // voltage in 10mV increments 1.1ref, 560 and 10k resistors
    uint32_t v = TakeADC(VOLTAGE, p);

    if(voltage_mode)
        // 14135 / 3584 = 100.0/1024*10280/280*1.1
        v = v * 14135 / 3584 / 16;
    else
        // 1815 / 896 = 100.0/1024*10560/560*1.1
        v = v * 1815 / 896 / 16;

    return v;
}

uint16_t TakeTemp(uint8_t index, uint8_t p)
{
    uint32_t v = TakeADC(index, p);
    // thermistors are 100k resistor to 5v, and 10k NTC to GND with 1.1v ref
    // V = 1.1 * v / 1024
    // V = R / (R + 100k) * 5.0
    // x = 1.1 / 5.0 * v / 1024
    // R = 100k * x / (1 - x)
    uint32_t R = 100061 * v / (74464 - v);  // resistance in ohms
    // T0 = 298, B = 3905, R0 = 10000
    // T = 1 / (1 / T0 + 1/B * ln(R/R0))
    // T = 1.0 / (1.0 / 298.0 + 1/3905.0 * math.log(R/10000.0)) - 273.0

    // a reasonable approximation to avoid floating point math over our range:
    // within 2 degrees from 30C to 80C
    // T = 300000/(R+2600) + 2

    // temperature in hundreths of degrees
    return 30000000/(R+2600) + 200;
}

uint16_t TakeRudder(uint8_t p)
{
    // 16 bit value for rudder
    return TakeADC(RUDDER, p) * 4;
}

#if 0
uint16_t TakeInternalTemp(uint8_t p)
{
    uint32_t v = TakeADC(INTERNAL_TEMP, p);
    // voltage = v * 1.1 / 1023
    // 25C = 314mV, 85C = 380mV
    // T = 909*V - 260
    return 611 * v / 100 - 26000;
}
#endif

ISR(WDT_vect)
{
    wdt_reset();
    wdt_disable();
    disengage();
    delay(50);
    detach();

    asm volatile ("ijmp" ::"z" (0x0000));
}

ISR(TIMER1_OVF_vect) __attribute__((naked));
ISR(TIMER1_OVF_vect)
{
    asm volatile ("push r24"); // this register used to read TIMSK1

    if(TIMSK1 & _BV(TOIE1)) {
        if(TIMSK1 & _BV(OCIE1A)) {
            asm volatile ("nop"); // balance 2 cycle delay of rjmp
            asm volatile ("nop"); // to get symmetric drive times
            a_top_off;
            dead_time;
            a_bottom_on;
            asm volatile ("pop r24");  // no need to rjmp
            asm volatile ("reti");
        } else {
            b_top_off;
            dead_time;
            b_bottom_on;
        }
    }
    
    asm volatile ("pop r24");
    asm volatile ("reti");
}

// use naked interrupts for maximum reaction
// this improves the upper range especially at
// 16khz with only 4mhz clock
ISR(TIMER1_COMPA_vect) __attribute__((naked));
ISR(TIMER1_COMPA_vect)
{
    a_bottom_off;
    b_top_off;
    dead_time;
    a_top_on;
    b_bottom_on;
    asm volatile ("reti");
}

ISR(TIMER1_COMPB_vect) __attribute__((naked));
ISR(TIMER1_COMPB_vect)
{
    b_bottom_off;
    a_top_off;
    dead_time;
    b_top_on;
    a_bottom_on;
    asm volatile ("reti");
}

void process_packet()
{
    flags |= SYNC;
    uint16_t value = in_bytes[1] | in_bytes[2]<<8;
    switch(in_bytes[0]) {
    case REPROGRAM_CODE:
    {
        // jump to bootloader
        asm volatile ("ijmp" ::"z" (0x3c00));
        //goto *0x3c00;
    } break;
    case RESET_CODE:
        // reset overcurrent flag
        flags &= ~OVERCURRENT;
        break;
    case COMMAND_CODE:
        timeout = 0;
        if(serialin < 12)
            serialin+=4; // output at input rate
        if(value > 2000);
            // unused range, invalid!!!
            // ignored
        else if(flags & (OVERTEMP | OVERCURRENT | BADVOLTAGE));
            // no command because of overtemp or overcurrent or badvoltage
        else if((flags & (FWD_FAULTPIN | MAX_RUDDER)) && value > 1000)
            stop();
            // no forward command if fwd fault
        else if((flags & (REV_FAULTPIN | MIN_RUDDER)) && value < 1000)
            stop();
            // no reverse command if fwd fault
        else {
            command_value = value;
            engage();
        }
        break;
    case MAX_CURRENT_CODE: { // current in units of 10mA
        unsigned int max_max_current = low_current ? 2000 : 4000;
        if(value > max_max_current) // maximum is 20 or 40 amps
            value = max_max_current;

        max_current = value;
    } break;
    case MAX_CONTROLLER_TEMP_CODE:
        if(value > 10000) // maximum is 100C
            value = 10000;
        max_controller_temp = value;
        break;
    case MAX_MOTOR_TEMP_CODE:
        if(value > 10000) // maximum is 100C
            value = 10000;
        max_motor_temp = value;
        break;
    case RUDDER_RANGE_CODE:
        rudder_max = in_bytes[1];
        rudder_min = in_bytes[2];
        break;
    case DISENGAGE_CODE:
        if(serialin < 12)
            serialin+=4; // output at input rate
        disengage();
        break;
    case MAX_SLEW_CODE: {
        max_slew_speed = in_bytes[1];
        max_slew_slow = in_bytes[2];

        // if set at the end of range (up to 255)  no slew limit
        if(max_slew_speed > 250)
            max_slew_speed = 250;
        if(max_slew_slow > 250)
            max_slew_slow = 250;

        // must have some slew
        if(max_slew_speed < 1)
            max_slew_speed = 1;
        if(max_slew_slow < 1)
            max_slew_slow = 1;
    } break;

    case EEPROM_READ_CODE:
        if(eeprom_read_addr == eeprom_read_end) {
            eeprom_read_addr = in_bytes[1];
            eeprom_read_end = in_bytes[2];
        }        
    break;

    case EEPROM_WRITE_CODE:
        eeprom_update_8(in_bytes[1], in_bytes[2]);
    break;
    }
}

void loop()
{
    wdt_reset(); // strobe watchdog

    // did timer2 pass 78?
    // Timer2 ticks at 3906hz (4mhz), so this is ~50hz
    if(TCNT2 > 78) {
        if(flags & ENGAGED) {
            static uint8_t update_d;
            if(++update_d >= 4/DIV_CLOCK) {
                update_command(); // 30hz
                update_d = 0;
            }
        }

        timeout++;
        TCNT2 -= 78;
    }

    if(timeout == 120/DIV_CLOCK)
        disengage();

    if(timeout >= 128/DIV_CLOCK) // detach 60 ms later so esc gets stop
        detach();

    // wait for characters
    // boot powered down, wake on data
#if 0 // hardly worth it
    if(!Serial.available()) {
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_enable();
        sleep_cpu();
    }
#endif
    // serial input
    while(Serial.available()) {
      uint8_t c = Serial.read();
      if(sync_b < 3) {
          in_bytes[sync_b] = c;
          sync_b++;
      } else {
          if(c == crc8(in_bytes, 3)) {
              if(in_sync_count >= 2) { // if crc matches, we have a valid packet
                  process_packet();
              } else
                  in_sync_count++;

              sync_b = 0;
              flags &= ~INVALID;
          } else {
              // invalid packet
              flags &= ~SYNC;
              stop(); //disengage();
              in_sync_count = 0;
              packet_count = 0;
              in_bytes[0] = in_bytes[1];
              in_bytes[1] = in_bytes[2];
              in_bytes[2] = c;
              flags |= INVALID;
          }
          break;
      }
    }

    // test fault pins
    if(!digitalRead(fwd_fault_pin)) {
        stop_fwd();
        flags |= FWD_FAULTPIN;
    } else
      flags &= ~FWD_FAULTPIN;

    if(!digitalRead(rev_fault_pin)) {
        stop_rev();
        flags |= REV_FAULTPIN;
    } else
      flags &= ~REV_FAULTPIN;

    // test current
    const int react_count = 686; // need 686 readings for 0.1s reaction time
    if(CountADC(CURRENT, 1) > react_count) {
        uint16_t amps = TakeAmps(1);
        if(amps >= max_current) {
            stop();
            faults |= OVERCURRENT;
        } else
            faults &= ~OVERCURRENT;
    }

    if(CountADC(VOLTAGE, 1) > react_count) {
        uint16_t volts = TakeVolts(1);
        if(volts >= 1800 && !voltage_mode && !voltage_sense) {
            // switch to higher voltage
            voltage_mode = 1; // higher voltage
            digitalWrite(voltage_sense_pin, LOW);
            pinMode(voltage_sense_pin, OUTPUT);
            max_voltage = 3600; // 36 v max
            delay(2);
            TakeVolts(0); // clear readings
            TakeVolts(1);
        } else
        /* voltage must be between 9 and max voltage */
        if(volts <= 900 || volts >= max_voltage) {
            stop();
            flags |= BADVOLTAGE;
        } else
            flags &= ~BADVOLTAGE;
    }

    flags |= faults;

    // test over temp
    const int temp_react_count = 200;
    if(CountADC(CONTROLLER_TEMP, 1) > temp_react_count &&
       CountADC(MOTOR_TEMP, 1) > temp_react_count) {
        uint16_t controller_temp = TakeTemp(CONTROLLER_TEMP, 1);
        uint16_t motor_temp = TakeTemp(MOTOR_TEMP, 1);
        if(controller_temp >= max_controller_temp || motor_temp > max_motor_temp) {
            stop();
            flags |= OVERTEMP;
        } else
            flags &= ~OVERTEMP;

        // 100C is max allowed temp, 117C is max measurable.
        // 110C indicates software fault
        if(controller_temp > 11000) {
            stop();
            asm volatile ("ijmp" ::"z" (0x0000)); // attempt soft reset
        }
    }

    const int rudder_react_count = 160; // approx 0.2 second reaction
    if(CountADC(RUDDER, 1) > rudder_react_count) {
        uint16_t v = TakeRudder(1);
        if(rudder_sense) {
            uint16_t w = v >> 8;
            if(w < rudder_min) {
                stop_rev();
                flags |= MIN_RUDDER;
            } else
                flags &= ~MIN_RUDDER;
            if(w > rudder_max) {
                stop_fwd();
                flags |= MAX_RUDDER;
            } else
                flags &= ~MAX_RUDDER;
            if(v < 1024 || v > 65472 - 1024)
                rudder_sense = 0;
        } else {
            if(v > 1536 && v < 65472 - 1536)
                rudder_sense = 1;
            flags &= ~(MIN_RUDDER | MAX_RUDDER);
        }
    }

    // output 1 byte
    switch(out_sync_b) {
    case 0:
        // match output rate to input rate
        if(serialin < 4)
            return;

        uint16_t v;
        uint8_t code;

        //  flags C R V C R ct C R mt flags  C  R  V  C  R EE  C  R mct flags  C  R  V  C  R  EE  C  R rr flags  C  R  V  C  R EE  C  R cc  C  R vc
        //  0     1 2 3 4 5  6 7 8  9    10 11 12 13 14 15 16 17 18  19    20 21 22 23 24 25  26 27 28 29    30 31 32 33 34 35 36 37 38 39 40 41 42
        switch(out_sync_pos++) {
        case 0: case 10: case 20: case 30:
            if(!low_current)
                flags |= CURRENT_RANGE;

            v = flags;
            code = FLAGS_CODE;
            break;
        case 1: case 4: case 7: case 11: case 14: case 17: case 21: case 24: case 27: case 31: case 34: case 37: case 40:
            if(CountADC(CURRENT, 0) < 50)
                return;

            v = TakeAmps(0);
            code = CURRENT_CODE;
            serialin-=4; // fix current output rate to input rate
            delay(1); // small dead time to break serial transmission
            if(packet_count < 255)
                packet_count++;
            break;
        case 2: case 5: case 8: case 12: case 15: case 18: case 22: case 25: case 28: case 32: case 35: case 38: case 41:
            if(CountADC(RUDDER, 0) < 10 || (!rudder_sense && out_sync_pos > 3))
                return;
            if(rudder_sense == 0)
                v = 65535; // indicate invalid rudder measurement
            else
                v = TakeRudder(0);
            code = RUDDER_SENSE_CODE;
            break;
        case 3: case 13: case 23: case 33:
            if(CountADC(VOLTAGE, 0) < 2)
                return;
            v = TakeVolts(0);
            code = VOLTAGE_CODE;
            break;
        case 6:
            if(CountADC(CONTROLLER_TEMP, 0)) {
                v = TakeTemp(CONTROLLER_TEMP, 0);
                code = CONTROLLER_TEMP_CODE;
                break;
            }
            return;
        case 9:
            if(CountADC(MOTOR_TEMP, 0)) {
                v = TakeTemp(MOTOR_TEMP, 0);
                if(v > 1200) { // below 12C means no sensor connected, or too cold to care
                    code = MOTOR_TEMP_CODE;
                    break;
                }
            }
            return;

        case 16: case 26: case 36: /* eeprom reads */
            if(eeprom_read_addr != eeprom_read_end) {
                uint8_t value;
                if(eeprom_read_8(eeprom_read_addr, value)) {
                    v = value << 8 | eeprom_read_addr;
                    eeprom_read_addr++;
                    code = EEPROM_VALUE_CODE;
                    out_sync_pos--; // fast eeprom read
                    break;
                }
                eeprom_read_addr++; // skip for now
            }
            return;
        default:
            return;
        }

        crcbytes[0] = code;
        crcbytes[1] = v;
        crcbytes[2] = v>>8;
        // fall through
    case 1: case 2:
        // write next
        Serial.write(crcbytes[out_sync_b]);
        out_sync_b++;
        break;
    case 3:
        // write crc of sync byte plus bytes transmitted
        Serial.write(crc8(crcbytes, 3));
        out_sync_b = 0;
        break;
    }
}
