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
This program just reads the analog rudder position on A0 and reports
the result over serial at 38400 baud.

The regular motor controller can do this and much more, this is for
a standalone rudder feedback.   Usually the motor controller should
be used for rudder feedback because of quicker reaction time
*/

// run at 4mhz instead of 16mhz to save power,
// and to be able to measure lower current from the shunt

#define DIV_CLOCK 4  // 1 for 16mhz, 2 for 8mhz, 4 for 4mhz

#define rudder_pin A0 // rudder on A0

// the mlx90316 used does not have a full range
uint16_t min_rudder_reading = 300;
uint16_t max_rudder_reading = 900;

float rudder_range = 80; // total angular range to report in rudder messages

#include <stdarg.h>
void debug(char *fmt, ... ){
    char buf[128]; // resulting string limited to 128 chars
    va_list args;
    va_start (args, fmt );
    vsnprintf(buf, 128, fmt, args);
    va_end (args);
    Serial.print(buf);
}

const uint8_t defmux = _BV(REFS0); // 5v (ratiometric)

void setup()
{
#if DIV_CLOCK==4
    CLKPR = _BV(CLKPCE);
    CLKPR = _BV(CLKPS1); // divide by 4
#elif DIV_CLOCK==2
    CLKPR = _BV(CLKPCE);
    CLKPR = _BV(CLKPS0); // divide by 2
#elif DIV_CLOCK!=1
    #error "invalid DIV_CLOCK"
#endif
    // Disable all interrupts
    cli();

/* Clear MCU Status Register. Not really needed here as we don't need to know why the MCU got reset. page 44 of datasheet */
    MCUSR = 0;

/* Disable and clear all Watchdog settings. Nice to get a clean slate when dealing with interrupts */

    WDTCSR = (1<<WDCE)|(1<<WDE);

    // interrupt in 1/4th second
    WDTCSR = (1<<WDIE) | (1<<WDP2);
    sei();

    Serial.begin(38400*DIV_CLOCK);

    // read fuses, and report this as flag if they are wrong
    uint8_t lowBits      = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
    uint8_t highBits     = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
    uint8_t extendedBits = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
    uint8_t lockBits     = boot_lock_fuse_bits_get(GET_LOCK_BITS);
    if(lowBits != 0xFF || highBits != 0xda ||
       (extendedBits != 0xFD && extendedBits != 0xFC) || lockBits != 0xCF)
        Serial.print("Warning, fuses set wrong, flash may become corrupted");


    set_sleep_mode(SLEEP_MODE_IDLE); // wait for serial

    pinMode(A0, INPUT);
    digitalWrite(A0, LOW);

    _delay_us(100); // time to settle

#if 1
    // setup adc
    DIDR0 = 0x3f; // disable all digital io on analog pins
    ADMUX = defmux; // 5v

    ADCSRA = _BV(ADEN) | _BV(ADIE); // enable adc with interrupts
#if DIV_CLOCK==4
//    ADCSRA |= _BV(ADPS2) | _BV(ADPS1); // divide clock by 64
    ADCSRA |= _BV(ADPS0); // divide clock by 64
//    ADCSRA |= _BV(ADPS0) |  _BV(ADPS1) | _BV(ADPS2); // divide clock by 128
#else
    ADCSRA |= _BV(ADPS0) |  _BV(ADPS1) | _BV(ADPS2); // divide clock by 128
#endif
    ADCSRA |= _BV(ADSC); // start conversion
#endif
}


enum {RUDDER, CHANNEL_COUNT};
const uint8_t muxes[] = {0};

volatile struct adc_results_t {
    uint32_t total;
    uint16_t count;
} adc_results[CHANNEL_COUNT];

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
    ADCSRA |= _BV(ADSC); // enable conversion
    uint16_t adcw = ADCW;
    if(++adc_cnt <= 3) // discard first few readings after changing channel
        return;

    if(adc_results[adc_counter].count < 4000) {
        adc_results[adc_counter].total += adcw;
        adc_results[adc_counter].count++;
    }

    if(adc_cnt < 16) // take more samples for rudder, if sampled
        return;

    // advance to next channel
    adc_cnt = 0;
    
    if(++adc_counter >= CHANNEL_COUNT)
        adc_counter=0;

    ADMUX = defmux | muxes[adc_counter];
}

// return value from 0-16368
uint16_t TakeADC(uint8_t index)
{
    uint32_t t, c;
    cli();
    t = adc_results[index].total;
    c = adc_results[index].count;
    adc_results[index].total = 0;
    adc_results[index].count = 0;
    sei();
    if(c == 0)
        return 0;

    uint32_t avg = 16 * t / c;
    return avg; // multiply by 16 to keep less significant bits
}

uint16_t TakeRudder()
{
    // 16 bit value for rudder
    return TakeADC(RUDDER) * 4;
}

ISR(WDT_vect)
{
    wdt_reset();
    wdt_disable();
    delay(50);
    asm volatile ("ijmp" ::"z" (0x0000));
}

uint8_t checksum(const char *buf)
{
    uint8_t cksum = 0;
    for(uint8_t i=0; i<strlen(buf); i++)
        cksum ^= buf[i];
    return cksum;
}

void send_nmea(const char *buf)
{
  char buf2[128];
  snprintf(buf2, sizeof buf2, "$%s*%02x\r\n", buf, checksum(buf));
  Serial.print(buf2);
}

static uint16_t lastt;
static float lp_rudder;

void loop()
{
    wdt_reset(); // strobe watchdog

    int t = millis();
    uint16_t period = t-lastt;
    if(period > 100/DIV_CLOCK) {// output after 100 milliseconds
        lastt += 100/DIV_CLOCK;
        //Serial.println(adc_results[0].count);
            
        uint16_t v = TakeRudder();

        uint16_t min_rudder_reading = 5000;
        uint16_t max_rudder_reading = 58000;

        char buf[20];
        if(v > min_rudder_reading && v < max_rudder_reading) {

            float rudder = float(v - min_rudder_reading) / (max_rudder_reading - min_rudder_reading);
            rudder =  (rudder - 0.5) * rudder_range;
            lp_rudder = .5*lp_rudder + .5*rudder; // slight filter

            snprintf(buf, sizeof buf, "ARRSA,%d.%02d,A,,", (int)lp_rudder,
                     (uint16_t)(lp_rudder*100.0)%100U);
        } else
            strcpy(buf, "ARRSA,,A,,");
        send_nmea(buf);
    }

    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sleep_cpu();
}
