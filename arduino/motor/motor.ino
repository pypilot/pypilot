/* Copyright (C) 2016 Sean D'Epagnier <seandepagnier@gmail.com>
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
motor controller either brushless or brushed, or
a regular RC servo

adc pin0 is a resistor divider to measure voltage
             allowing up to 20 volts (10k and 560 ohm, 1.1 volt reference)
             
adc pin1 goes to .1 ohm shunt to measure current
adc pin2 goes to 100k resistor to 5v and 10k NTC thermistor to gnd ctrl temp
adc pin3 goes to 100k resistor to 5v and 10k NTC thermistor to gnd motor temp

digital pin9 pwm output on arduino
digital pin2 esc programming input/output (with arduinousblinker script)

optional:
digital pin7 forward fault for optional switch to stop forward travel
digital pin8 reverse fault for optional switch to stop reverse travel
adc pin6 rudder sense

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

//#define ARDUINO_SERVO

#ifdef ARDUINO_SERVO // crappy resolution software servo
#include <Servo.h>
Servo myservo;  // create servo object to control a servo 
#endif

#define fwd_fault_pin 7 // use pin 7 for optional fault
#define rev_fault_pin 8 // use pin 7 for optional fault
#define shunt_sense_pin 4 // use pin 4 to specify shunt resistance
uint8_t shunt_resistance = 1;
// if switches pull this pin low, the motor is disengauged
// and will be noticed by the control program

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

void delay_us(long us)
{
    long i;
    for(i=0; i<us; i++)
      _delay_us(1);
}

//ISR(WDT_vect)
//{
//    sync_b = 0, in_sync_count = 0;
//    setup();
//}
uint8_t serialin;

void setup() 
{
    serialin = 0;
    // set up Serial library
    Serial.begin(38400);
            
    wdt_reset();
    wdt_disable();
    //wdt_enable(WDTO_4S);
    set_sleep_mode(SLEEP_MODE_IDLE); // wait for serial

    // setup adc
    DIDR0 = 0x3f; // disable all digital io on analog pins
//    ADMUX = _BV(REFS0); // external 5v
    ADMUX = _BV(REFS0)| _BV(REFS1) | _BV(MUX0); // 1.1v

    ADCSRA = _BV(ADEN) | _BV(ADIE); // enable adc with interrupts 
    ADCSRA |= _BV(ADPS0) |  _BV(ADPS1) | _BV(ADPS2); // divide clock by 128

    ADCSRA |= _BV(ADSC); // start conversion

    pinMode(fwd_fault_pin, INPUT);
    digitalWrite(fwd_fault_pin, HIGH); /* enable internal pullups */
    pinMode(rev_fault_pin, INPUT);
    digitalWrite(rev_fault_pin, HIGH); /* enable internal pullups */

    pinMode(shunt_sense_pin, INPUT);
    digitalWrite(shunt_sense_pin, HIGH); /* enable internal pullups */    
} 

enum commands {COMMAND_CODE = 0xc7, STOP_CODE = 0xe7, MAX_CURRENT_CODE = 0x1e, MAX_CONTROLLER_TEMP_CODE = 0xa4, MAX_MOTOR_TEMP_CODE = 0x5a, RUDDER_RANGE_CODE = 0xb6, REPROGRAM_CODE = 0x19, DISENGAUGE_CODE=0x68};
enum results {CURRENT_CODE = 0x1c, VOLTAGE_CODE = 0xb3, CONTROLLER_TEMP_CODE=0xf9, MOTOR_TEMP_CODE=0x48, RUDDER_SENSE_CODE=0xa7, FLAGS_CODE=0x8f};

enum {SYNC=1, OVERTEMP=2, OVERCURRENT=4, ENGAUGED=8, INVALID=16*1, FWD_FAULTPIN=16*2, REV_FAULTPIN=16*4, MIN_RUDDER=256*1, MAX_RUDDER=256*2};

uint8_t in_bytes[3];
uint8_t sync_b = 0, in_sync_count = 0;

uint8_t out_sync_b = 0, out_sync_pos = 0;
uint8_t crcbytes[3];
uint16_t max_current = 1;
uint16_t max_controller_temp = 7000; // 70C
uint16_t max_motor_temp = 7000; // 70C
uint16_t min_rudder_pos = 0, max_rudder_pos = 65472;

uint16_t flags = 0, faults = 0;

uint8_t timeout, rudder_sense = 0;

// command is from 0 to 2000 with 1000 being neutral
uint16_t lastpos = 1000;
void position(uint16_t value)
{
#ifdef ARDUINO_SERVO
    myservo.write(value * 9 / 100 - 12);
#else
  OCR1A = 1500 + value * 3 / 2;
#endif
  lastpos = value;
}

void stop()
{
    position(1000);
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

void disengauge()
{
    stop();
    delay(60); // ensure esc gets stop signal

    flags &= ~ENGAUGED;
#ifdef ARDUINO_SERVO
    myservo.detach();
#else
    TCCR1A=0;
    TCCR1B=0;
//    DDRB&=~_BV(PB1);
    pinMode(9, INPUT);
#endif

    TIMSK2 = 0;
}

unsigned int range = 40000;

void engauge()
{
    if(flags & ENGAUGED)
        return;

#ifdef ARDUINO_SERVO
    myservo.attach(9);
#else
    //Configure TIMER1
    TCCR1A|=_BV(COM1A1)|_BV(WGM11);        //NON Inverted PWM
    TCCR1B|=_BV(WGM13)|_BV(WGM12)|_BV(CS11); //PRESCALER=8 MODE 14(FAST PWM)
        
    ICR1=range-1;  //fPWM=50Hz (Period = 20ms Standard).

//    DDRB|=_BV(PB1);   //PWM Pins as Out
    pinMode(9, OUTPUT);
#endif

// use timer2 as timeout
    timeout = 0;
    TCCR2B = _BV(CS20) | _BV(CS21) | _BV(CS22);
    TIMSK2 = _BV(TOIE2);

    flags |= ENGAUGED;
}

// set hardware pwm to period of "(1500 + 1.5*value)/2" or "750 + .75*value" microseconds

enum {CURRENT, VOLTAGE, CONTROLLER_TEMP, MOTOR_TEMP, /*INTERNAL_TEMP, */RUDDER, CHANNEL_COUNT};
const uint8_t muxes[] = {_BV(MUX0), 0, _BV(MUX1), _BV(MUX0) | _BV(MUX1), /*_BV(MUX3),*/ _BV(MUX1) | _BV(MUX2)};

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
    uint16_t adcw = ADCW;
    ADCSRA |= _BV(ADSC); // enable conversion
    if(++adc_cnt <= 3) // discard first few readings after changing channel
        return;

    for(int i=0; i<2; i++) {
        if(adc_results[adc_counter][i].count < 4000) {
            adc_results[adc_counter][i].total += adcw;
            adc_results[adc_counter][i].count++;
        }
    }
    
    if(adc_counter == CURRENT) { // take more current measurements
        if(adc_cnt < 50)
            return;
    } else if(adc_counter == VOLTAGE) {
        if(adc_cnt < 7)
            return;
    } else if(adc_counter == RUDDER && rudder_sense)
        if(adc_cnt < 10) // take more samples for rudder, if sampled
            return;
    
    // advance to next channel
    adc_cnt = 0;
    if(++adc_counter >= CHANNEL_COUNT)
        adc_counter=0;
    ADMUX = defmux | muxes[adc_counter];
}

uint16_t CountADC(uint8_t index, uint8_t p)
{
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
    // current units of 10mA
    // 275 / 128 = 100.0/1024/.05*1.1   for 0.05 ohm shunt
    // 1375 / 128 = 100.0/1024/.01*1.1   for 0.01 ohm shunt
    uint32_t v = TakeADC(CURRENT, p);

    if(shunt_resistance)
        return v * 275 / 128 / 16;
    return v * 1375 / 128 / 16;
}

uint16_t TakeVolts(uint8_t p)
{
    // voltage in 10mV increments 1.1ref, 560 and 10k resistors
    // 1815 / 896 = 100.0/1024*10560/560*1.1  cli();
    uint32_t v = TakeADC(VOLTAGE, p);
    return v * 1815 / 896 / 16;
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

ISR(TIMER2_OVF_vect)
{
  if(++timeout == 60) // 1 second
      disengauge();
}

void process_packet()
{
    timeout = 0;
    flags |= SYNC;
    uint16_t value = in_bytes[1] | (in_bytes[2]<<8);
    switch(in_bytes[0]) {
    case REPROGRAM_CODE:
    {
        // jump to bootloader
        asm volatile ("ijmp" ::"z" (0x3c00));
        //goto *0x3c00;
    } break;  
    case STOP_CODE:
        stop();
        // reset overcurrent flag
        flags &= ~OVERCURRENT;
        break;
    case COMMAND_CODE:
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
            position(value);
            engauge();
        }
        break;
    case MAX_CURRENT_CODE: // current in units of 10mA
        if(value > 2000) // maximum is 20 amps
            value = 2000;
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
    case DISENGAUGE_CODE:
        disengauge();
        //delay(60);
        break;
    }
}

void loop()
{
#if 0 // debug
    uint16_t v = TakeVolts(0);
    uint16_t c = TakeAmps(0);
    uint16_t t1 = TakeTemp(CONTROLLER_TEMP, 0);
    uint16_t t2 = TakeTemp(MOTOR_TEMP, 0);
    //uint16_t t3 = TakeInternalTemp(0);

    debug("voltage %u current %u ct %u mt %u\r\n", v, c, t1, t2);
    delay(200); // small dead time to be safe
    return;
#endif
    // wait for characters
    // boot powered down, wake on data
    if(!Serial.available())
      set_sleep_mode(SLEEP_MODE_IDLE);

    // serial input
    while(Serial.available()) {
      uint8_t c = Serial.read();
      if(serialin < 12)
        serialin++; // output at input rate

      if(sync_b < 3) {
          in_bytes[sync_b] = c;
          sync_b++;
      } else {
          if(c == crc8(in_bytes, 3)) {
              if(in_sync_count >= 1) { // if crc matches, we have a valid packet
                  wdt_reset(); // strobe watchdog
                  process_packet();
              } else
                  in_sync_count++;
              sync_b = 0;
              flags &= ~INVALID;
          } else {
              // invalid packet
              flags &= ~SYNC;
              stop(); //disengauge();
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

    // test shunt type, if pin wired to ground, we have 0.01 ohm, otherwise 0.05 ohm
    shunt_resistance = digitalRead(shunt_sense_pin);
    
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
        delay(1); // small dead time to be safe
        // match output rate to input rate
        if(serialin < 4)
            return;
        
        uint16_t v;
        uint8_t code;

        //flg C R V C R ct C R V C  R  flags  C  R  V  C  R mt  C  R  V  C  R
        //0   1 2 3 4 5  6 7 8 9 10 11  12   13 14 15 16 17 18 19 20 21 22 23    
        switch(out_sync_pos++) {
        case 0: case 12:
            v = flags;
            code = FLAGS_CODE;
            break;
        case 1: case 4: case 7: case 10: case 13: case 16: case 19: case 22:
            if(CountADC(CURRENT, 0) < 50)
                return;
            v = TakeAmps(0);
            code = CURRENT_CODE;
            serialin-=4; // fix output rate to input rate
            break;
        case 2: case 5: case 8: case 11: case 14: case 17: case 20: case 23:
            if(CountADC(RUDDER, 0) < 10)
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
            if(!shunt_resistance && CountADC(CONTROLLER_TEMP, 0)) {
                v = TakeTemp(CONTROLLER_TEMP, 0);
                code = CONTROLLER_TEMP_CODE;
                break;
            }
            return;
        case 18:
            if(!shunt_resistance && CountADC(MOTOR_TEMP, 0)) {
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
    case 1:
    case 2:
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
