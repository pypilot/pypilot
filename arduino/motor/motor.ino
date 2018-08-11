/* Copyright (C) 2018 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

#include <Arduino.h>
#include <stdint.h>
#include <HardwareSerial.h>

/*
This program is meant to interface with pwm based
motor controller either brushless or brushed, or a regular RC servo

You may need to modify the source code to support different hardware

adc pin0 is a resistor divider to measure voltage
             allowing up to 20 volts (10k and 560 ohm, 1.1 volt reference)
adc pin1 goes to .01/.05 ohm shunt to measure current
adc pin2 goes to 100k resistor to 5v and 10k NTC thermistor to gnd ctrl temp
adc pin3 goes to 100k resistor to 5v and 10k NTC thermistor to gnd motor temp
adc pin6 rudder sense

unused analog pins should be grounded

digital pin4 specifies .01 ohm resistor if wired to ground, otherwise .05 ohm
digital pin6 determines RC pwm if 1, or Hbridge if 0.
if RC pwm:
   digital pin9 pwm output standard ESC (1-2 ms pulse every 20 ms)
           pin2 esc programming input/output (with arduinousblinker script)
if Hbridge
   digital pin2 and pin3 for low side, pin9 and pin10 for high side
digital pin13 is led on when engaged

optional:
digital pin7 forward fault for optional switch to stop forward travel
digital pin8 reverse fault for optional switch to stop reverse travel

Pins 4 and 5 determine the current sense as folows:
pin 5 determines high/low power  (20A or 60A max)

D4  D5
 1   1        .05 ohm, (or .001 ohm x 50 gain)
 0   1        .01 ohm
 1   0        .0005 ohm x 50 gain
 0   0        reserved


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

// run at 4mhz instead of 16mhz to save power,
// and to be able to measure lower current from the shunt
#define DIV_CLOCK

//#define HIGH_CURRENT   // high current uses 500uohm resistor and 50x amplifier
                         // 60amp range
//#define HIGH_CURRENT_OLD   // high current uses 500uohm resistor and 50x amplifier
// otherwise using shunt without amplification

#define rc_pwm_pin 6 // todo: use this pin to detect rc pwm
uint8_t rc_pwm = 1;  // remote control style servo

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

#ifdef DIV_CLOCK
#define dead_time _delay_us(2) // this is quite a long dead time
#else
#define dead_time _delay_us(8) // this is quite a long dead time
#endif

////// CRC
#include <avr/pgmspace.h>

const unsigned char crc8_table[256] PROGMEM
= {
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97,
    0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
    0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
    0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
    0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11,
    0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52,
    0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
    0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
    0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
    0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9,
    0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C,
    0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
    0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
    0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
    0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED,
    0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE,
    0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
    0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
    0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
    0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28,
    0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0,
    0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
    0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
    0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
    0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56,
    0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15,
    0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC
};

uint8_t crc8_byte(uint8_t old_crc, uint8_t byte){
    return pgm_read_byte(&crc8_table[old_crc ^ byte]);
}

uint8_t crc8_with_init(uint8_t init_value, uint8_t *pcBlock, uint8_t len)
{
    uint8_t crc = init_value;
    while (len--)
        crc = crc8_byte(crc, *pcBlock++);
    return crc;
}

uint8_t crc8(uint8_t *pcBlock, uint8_t len) {
    return crc8_with_init(0xFF, pcBlock, len);
}

////// ENDCRC

#include <avr/wdt.h>
#include <avr/sleep.h>

#define fwd_fault_pin 7 // use pin 7 for optional fault
#define rev_fault_pin 8 // use pin 7 for optional fault
// if switches pull this pin low, the motor is disengaged
// and will be noticed by the control program

#define shunt_sense_pin 4 // use pin 4 to specify shunt resistance
uint8_t shunt_resistance = 1;

#include <stdarg.h>
void debug(char *fmt, ... ){
    char buf[128]; // resulting string limited to 128 chars
    va_list args;
    va_start (args, fmt );
    vsnprintf(buf, 128, fmt, args);
    va_end (args);
    Serial.print(buf);
}

#include <util/delay.h>

uint8_t serialin;
void setup() 
{
#ifdef DIV_CLOCK
    CLKPR = _BV(CLKPCE);
    CLKPR = _BV(CLKPS1); // divide by 4
#endif
    // Disable all interrupts
    cli();

/* Clear MCU Status Register. Not really needed here as we don't need to know why the MCU got reset. page 44 of datasheet */
    MCUSR = 0;

/* Disable and clear all Watchdog settings. Nice to get a clean slate when dealing with interrupts */

    WDTCSR = (1<<WDCE)|(1<<WDE);

    // interrupt in 1/4th second
    WDTCSR = (1<<WDIE) | (1<<WDP2);

// Enable all interrupts.
    sei();

    // enable pullup on unused pins (saves .5 mA)
    pinMode(4, INPUT_PULLUP);
    pinMode(5, INPUT_PULLUP);
    pinMode(6, INPUT_PULLUP);
    pinMode(11, INPUT_PULLUP);
    pinMode(12, INPUT_PULLUP);
    
    serialin = 0;
    // set up Serial library
#ifdef DIV_CLOCK
    Serial.begin(38400*4);
#else
    Serial.begin(38400);
#endif

    set_sleep_mode(SLEEP_MODE_IDLE); // wait for serial
    // setup adc
    DIDR0 = 0x3f; // disable all digital io on analog pins
//    ADMUX = _BV(REFS0); // external 5v
    ADMUX = _BV(REFS0)| _BV(REFS1) | _BV(MUX0); // 1.1v
    ADCSRA = _BV(ADEN) | _BV(ADIE); // enable adc with interrupts 
#ifdef DIV_CLOCK
    ADCSRA |= _BV(ADPS2) | _BV(ADPS1); // divide clock by 64
//    ADCSRA |= _BV(ADPS0) |  _BV(ADPS1) | _BV(ADPS2); // divide clock by 128
#else
    ADCSRA |= _BV(ADPS0) |  _BV(ADPS1) | _BV(ADPS2); // divide clock by 128
#endif
    ADCSRA |= _BV(ADSC); // start conversion

    // status LED
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    
    if(rc_pwm) {
        digitalWrite(9, LOW); /* enable internal pullups */
        pinMode(9, OUTPUT);
    }
    
    pinMode(fwd_fault_pin, INPUT);
    digitalWrite(fwd_fault_pin, HIGH); /* enable internal pullups */
    pinMode(rev_fault_pin, INPUT);
    digitalWrite(rev_fault_pin, HIGH); /* enable internal pullups */


    pinMode(rc_pwm_pin, INPUT);
    digitalWrite(rc_pwm_pin, HIGH); /* enable internal pullups */    
    
    pinMode(shunt_sense_pin, INPUT);
    digitalWrite(shunt_sense_pin, HIGH); /* enable internal pullups */

    _delay_us(100);

    // test output type, pwm or h-bridge
    rc_pwm = digitalRead(rc_pwm_pin);
    
    // test shunt type, if pin wired to ground, we have 0.01 ohm, otherwise 0.05 ohm
    shunt_resistance = digitalRead(shunt_sense_pin);
}

enum commands {COMMAND_CODE = 0xc7, RESET_CODE = 0xe7, MAX_CURRENT_CODE = 0x1e, MAX_CONTROLLER_TEMP_CODE = 0xa4, MAX_MOTOR_TEMP_CODE = 0x5a, RUDDER_RANGE_CODE = 0xb6, REPROGRAM_CODE = 0x19, DISENGAGE_CODE=0x68, MAX_SLEW_CODE=0x71};

enum results {CURRENT_CODE = 0x1c, VOLTAGE_CODE = 0xb3, CONTROLLER_TEMP_CODE=0xf9, MOTOR_TEMP_CODE=0x48, RUDDER_SENSE_CODE=0xa7, FLAGS_CODE=0x8f};

enum {SYNC=1, OVERTEMP=2, OVERCURRENT=4, ENGAGED=8, INVALID=16*1, FWD_FAULTPIN=16*2, REV_FAULTPIN=16*4, MIN_RUDDER=256*1, MAX_RUDDER=256*2, CURRENT_RANGE=256*4};

uint8_t in_bytes[3];
uint8_t sync_b = 0, in_sync_count = 0;

uint8_t out_sync_b = 0, out_sync_pos = 0;
uint8_t crcbytes[3];
#if defined(HIGH_CURRENT) 
uint16_t max_current = 6000;
#else
uint16_t max_current = 2000;
#endif
uint16_t max_controller_temp = 7000; // 70C
uint16_t max_motor_temp = 7000; // 70C
uint16_t max_slew_speed = 100, max_slew_slow = 150; // 200 is full power in 1/10th of a second
uint16_t min_rudder_pos = 0, max_rudder_pos = 65472;

uint16_t flags = 0, faults = 0;

uint8_t timeout, rudder_sense = 0;

// command is from 0 to 2000 with 1000 being neutral
uint16_t lastpos = 1000;
void position(uint16_t value)
{
    if(rc_pwm)
#ifdef DIV_CLOCK
        OCR1A = 375 + value * 3 / 8;
#else
        OCR1A = 1500 + value * 3 / 2;
#endif
    else {
        if(value > 1040) {
#ifdef DIV_CLOCK
            OCR1A = (2022 - value) * 4;
#else
            OCR1A = (2016 - value) * 16;
#endif
            TIMSK1 = _BV(TOIE1) | _BV(OCIE1A);
        } else if(value < 960) {
#ifdef DIV_CLOCK
            OCR1B = (22 + value) * 4;
#else
            OCR1B = (16 + value) * 16;
#endif
            TIMSK1 = _BV(TOIE1) | _BV(OCIE1B);
        } else {
            TIMSK1 = _BV(TOIE1); // set brake
            a_top_off;
            b_top_off;
            dead_time;
            a_bottom_on;
            b_bottom_on;
        }
    }
    lastpos = value;
}

uint16_t command_value = 1000;
void stop()
{
    position(1000);
    command_value = 1000;
    lastpos = 1000;
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
    int16_t diff = command_value - cur_value;

    // limit motor speed change to stay within speed and slow slew rates
    if(cur_value > 1000) {
        if(diff > 0) {
            if(diff > speed_rate)
                diff = speed_rate;
        } else
            if(diff < -slow_rate)
                diff = -slow_rate;
    } else {
        if(diff < 0) {
            if(diff < -speed_rate)
                diff = -speed_rate;
        } else
            if(diff > slow_rate)
                diff = slow_rate;
    }
    position(cur_value + diff);
}

void disengage()
{
    stop();
    flags &= ~ENGAGED;
    timeout = 60; // detach in about 62ms
    digitalWrite(13, LOW); // status LED
}

void detach()
{
    if(rc_pwm) {
        TCCR1A=0;
        TCCR1B=0;
        while(digitalRead(9)); // wait for end of pwm if pulse is high
        TIMSK1 = 0;
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
    timeout = 64; // avoid overflow
}    

void engage()
{
    if(flags & ENGAGED)
        return;

    if(rc_pwm) {
        TCNT1 = 0x1fff;
        //Configure TIMER1
        TCCR1A=_BV(COM1A1)|_BV(WGM11);        //NON Inverted PWM
        TCCR1B=_BV(WGM13)|_BV(WGM12)|_BV(CS11); //PRESCALER=8 MODE 14(FAST PWM)
#ifdef DIV_CLOCK
        ICR1=10000;  //fPWM=50Hz (Period = 20ms Standard).
#else
        ICR1=40000;  //fPWM=50Hz (Period = 20ms Standard).
#endif

        TIMSK1 = _BV(TOIE1);
    //pinMode(9, OUTPUT);
    } else {
        //TCNT1 = 0x1fff;
        //Configure TIMER1
        TCCR1A=_BV(WGM11);        //NON Inverted PWM
        TCCR1B=_BV(WGM13)|_BV(WGM12)|_BV(CS10); //PRESCALER=0 MODE 14(FAST PWM)
#ifdef DIV_CLOCK
        ICR1=4000;  //fPWM=1khz
#else
        ICR1=16000;  //fPWM=1khz
#endif
        TIMSK1 = 0;
        a_top_off;
        a_bottom_off;
        b_top_off;
        b_bottom_off;

        pinMode(2, OUTPUT);
        pinMode(3, OUTPUT);
        pinMode(9, OUTPUT);
        pinMode(10, OUTPUT);
    }

// use timer2 as timeout
    timeout = 0;
#ifdef DIV_CLOCK
    TCCR2B = _BV(CS22) | _BV(CS21); // divide 256
#else
    TCCR2B = _BV(CS20) | _BV(CS21) | _BV(CS22); // divide 1024
#endif
    TIMSK2 = _BV(TOIE2);

    flags |= ENGAGED;

    update_command();
    digitalWrite(13, HIGH); // status LED
}

// set hardware pwm to period of "(1500 + 1.5*value)/2" or "750 + .75*value" microseconds

enum {CURRENT, VOLTAGE, CONTROLLER_TEMP, MOTOR_TEMP, /*INTERNAL_TEMP, */RUDDER, CHANNEL_COUNT};
const uint8_t muxes[] = {_BV(MUX0), 0, _BV(MUX1), _BV(MUX0) | _BV(MUX1), /*_BV(MUX3),*/
#if defined(HIGH_CURRENT) 
                         _BV(MUX2)
#else
                         _BV(MUX1) | _BV(MUX2)
#endif                         
};

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
    if(rc_pwm)
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
    ADMUX = defmux | muxes[adc_counter];
ret:;
    if(!rc_pwm)
        ADCSRA |= _BV(ADSC); // enable conversion
}

uint16_t CountADC(uint8_t index, uint8_t p)
{
//    if(index == CURRENT)
//        adc_results[index][p].count++;
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

#if defined(HIGH_CURRENT) 
   // high curront controller has .0005 ohm with 50x gain
    // 3663/511 = 100.0/1023/.0003/50*1.1
    // 1200/279 = 100.0/1023/.0005/50*1.1
    if(v > 16)
#ifdef DIV_CLOCK        
        v = v * 1200 / 279 / 16; // 420mA offset
#else
        v = v * 1200 / 279 / 16 + 82; // 820mA offset
#endif
    return v;
#elif defined(HIGH_CURRENT_OLD)
    // high curront controller has .001 ohm with 50x gain
    // 275/128 = 100.0/1023/.001/50*1.1
    return v * 275 / 128 / 16;
#else
    // current units of 10mA
    // 275 / 128 = 100.0/1024/.05*1.1   for 0.05 ohm shunt
    // 1375 / 128 = 100.0/1024/.01*1.1   for 0.01 ohm shunt
   if(shunt_resistance)
        return v * 275 / 128 / 16;

    if(v > 16)
#ifdef DIV_CLOCK        
        v = v * 1375 / 128 / 16 + 18; // 200mA minimum
#else
        v = v * 1375 / 128 / 16 + 55; // 550mA minimum
#endif
    return v;
#endif
}

uint16_t TakeVolts(uint8_t p)
{
    // voltage in 10mV increments 1.1ref, 560 and 10k resistors
    // 1815 / 896 = 100.0/1024*10560/560*1.1  cli();
    uint32_t v = TakeADC(VOLTAGE, p);
    return v * 1815 / 896 / 16 + 40;
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

static volatile uint8_t timer1_cnt;
ISR(TIMER1_OVF_vect)
{
    if(rc_pwm)
        update_command();
    else {
        if(lastpos > 1000) {
            a_top_off;
            dead_time;
            a_bottom_on;
        } else if(lastpos < 1000) {
            b_top_off;
            dead_time;
            b_bottom_on;
        }
        sei();
        if(++timer1_cnt == 20) { // update slew speeds at 50hz
            update_command();
            timer1_cnt = 0;
        }
    }
}

ISR(TIMER1_COMPA_vect)
{
    a_bottom_off;
    b_top_off;
    dead_time;
    a_top_on;
    b_bottom_on;
}

ISR(TIMER1_COMPB_vect)
{
    b_bottom_off;
    a_top_off;
    dead_time;
    b_top_on;
    a_bottom_on;
}

ISR(TIMER2_OVF_vect)
{
    sei();
    timeout++;
    if(timeout == 60)
        disengage();
    if(timeout >= 64) // detach 60 ms later so esc gets stop
        detach();
}

void process_packet()
{
    flags |= SYNC;
    uint16_t value = in_bytes[1] | (in_bytes[2]<<8);
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
        else if(flags & (OVERTEMP | OVERCURRENT));
            // no command because of overtemp or overcurrent
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
    case MAX_CURRENT_CODE: // current in units of 10mA
#if defined(HIGH_CURRENT) || defined(HIGH_CURRENT_OLD)
        if(value > 6000) // maximum is 60 amps
            value = 6000;
#else
        if(value > 2000) // maximum is 20 amps
            value = 2000;
#endif
        max_current = value;
        break;
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
        min_rudder_pos = 256*in_bytes[1];
        max_rudder_pos = 256*in_bytes[2];
        break;
    case DISENGAGE_CODE:
        if(serialin < 12)
            serialin+=4; // output at input rate
        disengage();
        break;
    case MAX_SLEW_CODE:
        max_slew_speed = in_bytes[1];
        max_slew_slow = in_bytes[2];

        // if set at the end of range (up to 255)  no slew limit
        if(max_slew_speed > 250)
            max_slew_speed = 1000;
        if(max_slew_slow > 250)
            max_slew_slow = 1000;

        // must have some slew
        if(max_slew_speed < 1) 
            max_slew_speed = 1;
        if(max_slew_slow < 1) 
            max_slew_slow = 1;
        break;
    }
}

void loop()
{
    wdt_reset(); // strobe watchdog

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
              if(in_sync_count >= 1) { // if crc matches, we have a valid packet
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
            asm volatile ("ijmp" ::"z" (0x0000));
        }
    }

    const int rudder_react_count = 160; // approx 0.2 second reaction
    if(CountADC(RUDDER, 1) > rudder_react_count) {
        uint16_t v = TakeRudder(1);
        if(rudder_sense) {
            if(v < min_rudder_pos) {
                stop_rev();
                flags |= MIN_RUDDER;
            } else
                flags &= ~MIN_RUDDER;
            if(v > max_rudder_pos) {
                stop_fwd();
                flags |= MAX_RUDDER;
            } else
                flags &= ~MAX_RUDDER;
            if(v < 256 || v > 65472 - 256)
                rudder_sense = 0;
        } else {
            if(v > 256 && v < 65472 - 256)
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

        //flg C R V C R ct C R V C  R  flags  C  R  V  C  R mt  C  R  V  C  R
        //0   1 2 3 4 5  6 7 8 9 10 11  12   13 14 15 16 17 18 19 20 21 22 23    
        switch(out_sync_pos++) {
        case 0: case 12:
#if defined(HIGH_CURRENT) 
            flags |= CURRENT_RANGE;
#endif
            v = flags;
            code = FLAGS_CODE;
            break;
        case 1: case 4: case 7: case 10: case 13: case 16: case 19: case 22:
            if(CountADC(CURRENT, 0) < 50) {
//                out_sync_pos--; // remain at current measurement (avoid output overflow)
                return;
            }
            v = TakeAmps(0);
            code = CURRENT_CODE;
            serialin-=4; // fix current output rate to input rate
            delay(1); // small dead time to break serial transmission
            break;
        case 2: case 5: case 8: case 11: case 14: case 17: case 20: case 23:
            if(CountADC(RUDDER, 0) < 10 || (!rudder_sense && out_sync_pos > 3))
                return;
            if(rudder_sense == 0)
                v = 65535; // indicate invalid rudder measurement
            else
                v = TakeRudder(0);
            code = RUDDER_SENSE_CODE;
            break;
        case 3: case 9: case 15: case 21:
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
        case 18:
            if(CountADC(MOTOR_TEMP, 0)) {
                v = TakeTemp(MOTOR_TEMP, 0);
                if(v > 1200) { // below 12C means no sensor connected, or too cold to care
                    code = MOTOR_TEMP_CODE;
                    break;
                }
            }
            return;
        default:
            out_sync_pos = 0;
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
