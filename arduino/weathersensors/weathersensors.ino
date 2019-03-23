/* Copyright (C) 2019 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

/* this program interfaces with the bmp280 pressure sensor and outputs
 calibrated signalk data over serial at 38400 baud
 */

/* anemometer wires

using potentiometer for wind direction
using a reed switch for wind speed

red    - gnd - pin 3 - potentiometer power A - reed switch A
yellow - 5v  - pin 5 - potentiometer power B
green  - a7  - pin 4 - potentiometer reading
black  - d2  - pin 2 -                         reed switch B

*/

#include <Arduino.h>
#include <stdint.h>
#include <avr/sleep.h>
#include <HardwareSerial.h>
#include "PCD8544.h"
#include <avr/boot.h>

extern "C" {
  #include <twi.h>
}

#define ANEMOMETER   // comment to show only baro graph
#define LCD
#define DAVIS

#define LCD_BL_HIGH  // if backlight pin is high rather than gnd
#define FARENHIET

#ifdef LCD
static PCD8544 lcd(13, 11, 8, 7, 4);
#endif

const int analogInPin = A7;  // Analog input pin that the potentiometer is attached to
const int analogLightPin = A6;
const int analogBacklightPin = 9;

uint8_t have_bmp280 = 0;
uint8_t bmX280_tries = 10;  // try 10 times to configure
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

void bmX280_setup()
{
  Serial.println("bmX280 setup");

  // NOTE:  local version of twi does not enable pullup as
  // bmX_280 device is 3.3v and arduino runs at 5v

  twi_init();

  // incase arduino version (although pullups will put
  // too high voltage for short time anyway....
  pinMode(SDA, INPUT);
  pinMode(SCL, INPUT);
  delay(1);

  have_bmp280 = 0;
  bmX280_tries--;

  uint8_t d[24];
  d[0] = 0xd0;

  if(twi_writeTo(0x76, d, 1, 1, 1) == 0 &&
     twi_readFrom(0x76, d, 1, 1) == 1 &&
     d[0] == 0x58)
      have_bmp280 = 1;
  else {
      Serial.print("bmp280 not found: ");
      Serial.println(d[0]);
      // attempt reset command
      //d[0] = 0xe0;
      //d[1] = 0xb6;
      //twi_writeTo(0x76, d, 2, 1, 1);
      TWCR &= ~(_BV(TWEN));
      return;
  }

  d[0] = 0x88;
  if(twi_writeTo(0x76, d, 1, 1, 1) != 0)
      have_bmp280 = 0;

  uint8_t c = twi_readFrom(0x76, d, 24, 1);
  if(c != 24) {
      Serial.println("bmp280 failed to read calibration");
      have_bmp280 = 0;
  }
      
  dig_T1 = d[0] | d[1] << 8;
  dig_T2 = d[2] | d[3] << 8;
  dig_T3 = d[4] | d[5] << 8;
  dig_P1 = d[6] | d[7] << 8;
  dig_P2 = d[8] | d[9] << 8;
  dig_P3 = d[10] | d[11] << 8;
  dig_P4 = d[12] | d[13] << 8;
  dig_P5 = d[14] | d[15] << 8;
  dig_P6 = d[16] | d[17] << 8;
  dig_P7 = d[18] | d[19] << 8;
  dig_P8 = d[20] | d[21] << 8;
  dig_P9 = d[22] | d[23] << 8;

  
#if 0
  Serial.println("bmp280 pressure compensation:");
  Serial.println(dig_T1);
  Serial.println(dig_T2);
  Serial.println(dig_T3);
  Serial.println(dig_P1);
  Serial.println(dig_P2);
  Serial.println(dig_P3);
  Serial.println(dig_P4);
  Serial.println(dig_P5);
  Serial.println(dig_P6);
  Serial.println(dig_P7);
  Serial.println(dig_P8);
  Serial.println(dig_P9);
#endif  

  // b00011111  // configure
  d[0] = 0xf4;
  d[1] = 0xff;
  if(twi_writeTo(0x76, d, 2, 1, 1) != 0)
      have_bmp280 = 0;
}

volatile unsigned int rotation_count;
volatile uint16_t lastperiod;

void isr_anemometer_count()
{
    uint8_t pin = digitalRead(2);

    static uint16_t lastt, lastofft;
    static uint8_t lastpin;
    int t = millis();

    if(!pin && lastpin) {
        uint16_t period = t-lastt;
        uint16_t offperiod = t-lastofft;

#ifdef DAVIS
        if(period > 20)
#else
        if(offperiod > 20 && // debounce, at least for less than 120 knots of wind
           offperiod > period/2 && offperiod < period/10*9) // test good reading
#endif
        {
            lastt = t;            
            lastperiod += period;
            rotation_count++;
        } else { // bad reading, reset

        }
    }
    lastpin = pin;
    lastofft = t;
}

#include <avr/wdt.h>
#include <avr/eeprom.h>

struct eeprom_data_struct {
    char signature[6];
    int16_t wind_min_reading, wind_max_reading;
} eeprom_data;

void setup()
{
  cli();
  MCUSR = 0;
  WDTCSR = (1<<WDCE) | (1<<WDE);
  WDTCSR = (1<<WDIE) | (1<<WDP2) | (1<<WDP1); // interrupt in 1 second
  sei();

  // default values
  char signature[] =  "arws13";
  memcpy(eeprom_data.signature, signature, sizeof eeprom_data.signature);
  eeprom_data.wind_min_reading = 300;
  eeprom_data.wind_max_reading = 650;
  
  // read eeprom and determine if it is valid
  struct eeprom_data_struct ram_eeprom;
  eeprom_read_block(&ram_eeprom, 0, sizeof ram_eeprom);
  if(memcmp(ram_eeprom.signature, signature, sizeof ram_eeprom.signature) != 0)
      eeprom_update_block(&eeprom_data, 0, sizeof eeprom_data);
  else if(ram_eeprom.wind_min_reading > 0 && // ensure somewhat sane range
         ram_eeprom.wind_max_reading < 1024 &&
         ram_eeprom.wind_min_reading < ram_eeprom.wind_max_reading-100)
      memcpy(&eeprom_data, &ram_eeprom, sizeof eeprom_data);
  
  Serial.begin(38400);  // start serial for output

  // read fuses, and report this as flag if they are wrong
    uint8_t lowBits      = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
    uint8_t highBits     = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
    uint8_t extendedBits = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
    uint8_t lockBits     = boot_lock_fuse_bits_get(GET_LOCK_BITS);
    if(lowBits != 0xFF || highBits != 0xda ||
       (extendedBits != 0xFD && extendedBits != 0xFC) || lockBits != 0xCF)
        Serial.print("Warning, fuses set wrong, flash may become corrupted");

  bmX280_setup();

#ifdef ANEMOMETER
  attachInterrupt(0, isr_anemometer_count, CHANGE);
  pinMode(analogInPin, INPUT);
  pinMode(2, INPUT_PULLUP);
#endif

  //analogRead(analogInPin); // select channel
  ADMUX = _BV(REFS0) | 6;
  ADCSRA |= _BV(ADIE);
  ADCSRA |= _BV(ADSC);   // Set the Start Conversion flag.

#ifdef LCD
    // PCD8544-compatible displays may have a different resolution...
  lcd.begin(84, 48);
  pinMode( analogLightPin, INPUT);
#endif
}

ISR(WDT_vect)
{
    wdt_reset();
    wdt_disable();
    delay(1);
    asm volatile ("ijmp" ::"z" (0x0000));
}

static volatile uint8_t adcchannel;
static volatile uint32_t adcval[4];
static volatile int16_t adccount[4];

ISR(ADC_vect)
{
    ADCSRA |= _BV(ADSC);   // Set the Start Conversion flag.
#ifdef ANEMOMETER
    uint16_t adcw = ADCW;
    if(adcchannel == 0) {
        adccount[0]++;
        // backlight sensor
        if(adcchannel == 0 && adccount[0] >= 1) {
            adcval[0] = adcw;
            ADMUX = _BV(REFS0) | 7, adcchannel = 1;
        }
    } else {
        uint8_t b = adcw < 448 ? 1 : adcw < 576 ? 2 : 3;
        if(adccount[b] < 4000) {
            adccount[b]++;
            if(adccount[b] > 0)
                adcval[b] += adcw;
        }
    }
#else
    adcval[0] = ADCW;
#endif
}

int32_t t_fine;
int32_t bmp280_compensate_T_int32(int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t bmp280_compensate_P_int64(int32_t adc_P)
{
    uint64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (uint64_t)dig_P6;
    var2 = var2 + ((var1*(uint64_t)dig_P5)<<17);
    var2 = var2 + (((uint64_t)dig_P4)<<35);
    var1 = ((var1 * var1 * (uint64_t)dig_P3)>>8) + ((var1 * (uint64_t)dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((uint64_t)dig_P1)>>33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((uint64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((uint64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((uint64_t)dig_P7)<<4);
    return (uint32_t)p;
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

int lcd_update;
float wind_dir, wind_speed, wind_speed_30;

void read_anemometer()
{
    static float lpdir;
    const float lp = .1; // lowpass filter constant

    if(adccount[1] < 100 && adccount[2] < 100 && adccount[3] < 100)
        return; // not enough data
    
    cli();
    uint32_t val[3];
    int16_t count[3];
    for(int i=0; i<3; i++) {
        val[i] = adcval[i+1];
        count[i] = adccount[i+1];
    }
    // reset the adc
    for(int i=0; i<4; i++) {
        adcval[i] = 0;
        adccount[i] = -4;
    }

#ifdef LCD
    // read from backlight sense
    adcchannel = 0;
    ADMUX = _BV(REFS0) | 6;
#else
    ADMUX = _BV(REFS0) | 7, adcchannel = 1;
#endif
    sei();

    // read the analog in value:
    int16_t sensorValue = 0;
    // discard this since we crossed zero and read
    // possibly invalid data
    if(count[0] > 0 && count[2] > 0) {
        //Serial.println("CROSS!!!!");
        return;
    }

    uint32_t tval = 0;
    int16_t tcount = 0;
    for(int i=0; i<3; i++) {
        if(count[i] > 64) {
            tval += val[i];
            tcount += count[i];
        }
    }


    if(tcount < 64) {
        Serial.println(count[0]);
        Serial.println(count[1]);
        Serial.println(count[2]);
        Serial.println(tcount);
        //Serial.println("Not enough data!!");
        return; // not enough data
    }

    sensorValue = tval / tcount; // average data

#ifdef DEBUG
    Serial.print(sensorValue);
    Serial.print("  ");
    Serial.print(eeprom_data.wind_min_reading);
    Serial.print(" ");
    Serial.println(eeprom_data.wind_max_reading);
#endif
      
    // make sure the value is sane
    if(sensorValue < 0 || sensorValue > 1023) {
        Serial.println("invalid range: program error");
        return;
    }

    if(sensorValue < eeprom_data.wind_min_reading / 2 && eeprom_data.wind_min_reading >= 40)
        lpdir = -1; // invalid
    else
    {
        float dir;

        int noise = 5;
        if(sensorValue < eeprom_data.wind_min_reading - noise) {
            // new minimum
            eeprom_data.wind_min_reading = sensorValue + noise;
            eeprom_update_block(&eeprom_data, 0, sizeof eeprom_data);
        } else if(sensorValue < eeprom_data.wind_min_reading)
            sensorValue = eeprom_data.wind_min_reading;
        else if(sensorValue > eeprom_data.wind_max_reading + noise) {
            eeprom_data.wind_max_reading = sensorValue - noise;
            eeprom_update_block(&eeprom_data, 0, sizeof eeprom_data);
        }
        else if(sensorValue > eeprom_data.wind_max_reading)
            sensorValue = eeprom_data.wind_max_reading;

        // compensate 13 degree deadband in potentiometer
        if(eeprom_data.wind_min_reading < 40 || eeprom_data.wind_max_reading > 1000)
            dir = (sensorValue + 13) * .34;

        else
            dir = float(sensorValue - eeprom_data.wind_min_reading)
                / (eeprom_data.wind_max_reading - eeprom_data.wind_min_reading) * 360.0;

        if(lpdir - dir > 180)
            dir += 360;
        else if(dir - lpdir > 180)
            dir -= 360;

        if(fabs(lpdir - dir) < 180) // another test which should never fail
            lpdir = lp*dir + (1-lp)*lpdir;

        if(lpdir >= 360)
            lpdir -= 360;
        else if(lpdir < 0)
            lpdir += 360;
    }

    uint16_t time = millis();
    static uint16_t last_time;
    uint16_t dt = time - last_time;
    if(dt >= 100) { // output every 100ms
        last_time += 100;

        cli();
        uint16_t period = lastperiod;
        uint16_t count = rotation_count;
        rotation_count = 0;
        lastperiod = 0;
        sei();
        
        static uint16_t nowindcount;
        static float knots = 0, lastnewknots = 0;;
        const int nowindtimeout = 30;
        if(count) {
            if(nowindcount!=nowindtimeout) {
                float newknots = .868976 * 2.25 * 1000 * count / period;
                if(fabs(lastnewknots/newknots-1) < .15) // if changing too fast, maybe bad reading
                    knots = newknots;

                lastnewknots = newknots;
            }

            nowindcount = 0;
        } else {
            if(nowindcount<nowindtimeout)
                nowindcount++;
            else
                knots = lastnewknots = 0;
        }

        char buf[128];
        if(lpdir >= 0)
            snprintf(buf, sizeof buf, "ARMWV,%d.%02d,R,%d.%02d,N,A", (int)lpdir, (uint16_t)(lpdir*100.0)%100U, (int)knots, (int)(knots*100)%100);
        else // invalid wind direction (no magnet?)
            snprintf(buf, sizeof buf, "ARMWV,,R,%d.%02d,N,A", (int)knots, (int)(knots*100)%100);
        send_nmea(buf);
        
        wind_dir = lpdir;
        wind_speed = knots;
        wind_speed_30 = wind_speed_30*299.0/300.0 + wind_speed/300.0;
        lcd_update = 1;
    }
}

uint32_t pressure, temperature;
int32_t pressure_comp, temperature_comp;
int bmp280_count;
void read_pressure_temperature()
{
    if(!bmX280_tries)
        return;

    uint8_t buf[6] = {0};
    buf[0] = 0xf7;
    uint8_t r = twi_writeTo(0x76, buf, 1, 1, 1);
    if(r) {
        if(have_bmp280) {
            Serial.print("bmp280 twierror ");
            Serial.println(r);
        }
    }

    uint8_t c = twi_readFrom(0x76, buf, 6, 1);
    if(c != 6) {
        if(have_bmp280)
            Serial.println("bmp280 failed to read 6 bytes from bmp280");
    }

    int32_t p, t;
    
    p = (int32_t)buf[0] << 16 | (int32_t)buf[1] << 8 | (int32_t)buf[2];
    t = (int32_t)buf[3] << 16 | (int32_t)buf[4] << 8 | (int32_t)buf[5];
    
    if(t == 0 || p == 0)
        have_bmp280 = 0;

    pressure += p >> 2;
    temperature += t >> 2;
    bmp280_count++;

    if(!have_bmp280) {
        if(bmp280_count == 256) {
            bmp280_count = 0;
            /* only re-run setup when count elapse */
            bmX280_setup();
        }
        return;
    }

    if(bmp280_count == 1024) {
        bmp280_count = 0;
        pressure >>= 12;
        temperature >>= 12;
    
        temperature_comp = bmp280_compensate_T_int32(temperature);
        pressure_comp = bmp280_compensate_P_int64(pressure) >> 8;
        pressure = temperature = 0;
  
        char buf[128];
        int ap = pressure_comp/100000;
        uint32_t rp = pressure_comp - ap*100000UL;
        snprintf(buf, sizeof buf, "ARMDA,,,%d.%05ld,B,,,,,,,,,,,,,,,,", ap, rp);
        send_nmea(buf);

        int a = temperature_comp / 100;
        int r = temperature_comp - a*100;
        snprintf(buf, sizeof buf, "ARMTA,%d.%02d,C", a, abs(r));
        send_nmea(buf);

        lcd_update = 1;
    }
}

void read_light()
{
    cli();
    int16_t light = adcval[0];
    sei();
    // filter and map light value
    static int lastlight, lighton;
    light = (light + 31*lastlight) / 32;
    lastlight = light;

    if(lighton) {
        if(light > 900)
            lighton = 0;
    } else {
        if(light < 800)
            lighton = 1;
    }
#ifdef LCD_BL_HIGH
    int pwm = lighton ? 90 : 0; // backlight expects 3v3 but we have 5v!
                                // do not set above 160 or may damage backlight!!
#else
    int pwm = lighton ? 120 : 255;
#endif
    #if 0
    Serial.print("light ");
    Serial.print(light);
    Serial.print(" pwm ");
    Serial.print(pwm);
    Serial.println("");
    #endif
    analogWrite(analogBacklightPin, pwm);
}

static uint16_t last_lcd_updatetime, last_lcd_texttime;
static char status_buf[4][16];
void draw_anemometer()
{
#ifdef LCD
    if(!lcd_update)
        return;

    uint16_t time = millis();
    uint16_t dt = time - last_lcd_updatetime;
    if(dt < 100) // don't update faster than 2frames per second
        return;

    last_lcd_updatetime = time;
    
    lcd_update = 0;

    dt = time - last_lcd_texttime;
    if(dt > 700) { // don't update text so fast
        int a, r;
        last_lcd_texttime = time;

        if(wind_dir>=0)
            snprintf(status_buf[0], sizeof status_buf[0], "%02d", (int) round(wind_dir));

        snprintf(status_buf[1], sizeof status_buf[1], "%02d", (int) round(wind_speed));

        lcd.rectangle(0, 46, 48, 83, 0); // clear text
        // draw wind speed
        lcd.setfont(3);
        lcd.setpos(0, 38);
        lcd.print(status_buf[1]);

        // draw 30 second average wind speed
        snprintf(status_buf[1], sizeof status_buf[1], "%02d", (int) round(wind_speed_30));
        lcd.setfont(2);
        lcd.setpos(29, 45);
        lcd.print(status_buf[1]);

        lcd.setfont(0);
        lcd.setpos(0, 61);
        a = pressure_comp/1e2, r = pressure_comp - a*1e2;
        a = snprintf(status_buf[2], sizeof status_buf[2], "%d", a);
        lcd.print(status_buf[2]);
        lcd.rectangle(a*7+3, 73, a*7+4, 73, 255); // draw decimal
        snprintf(status_buf[2], sizeof status_buf[2], "%02d", r);
        lcd.setpos(a*7+6, 61);
        lcd.print(status_buf[2]);

        char unit = 'C';
#ifdef FARENHIET
        unit = 'F';
        temperature_comp = temperature_comp*9/5+3200;
#endif
        
        a = temperature_comp / 100;
        r = temperature_comp - a*100;
        snprintf(status_buf[3], sizeof status_buf[3], "%d.%02d%c", a, abs(r), unit);
        
        lcd.setfont(0);
        lcd.setpos(1, 71);
        lcd.print(status_buf[3]);
    }
    
    lcd.rectangle(0, 0, 48, 45, 0); // clear compass
    // draw direction dial for wind
    lcd.circle(24, 22, 22, 255);

    if(wind_dir >= 0) {
        float wind_rad = wind_dir/180.0*M_PI;
        int r = 22;
        int x = r*sin(wind_rad);
        int y = -r*cos(wind_rad);
        for(int s=1; s<3; s++) {
            int xp = s*cos(wind_rad);
            int yp = s*sin(wind_rad);
            lcd.line(24+xp, 22+yp, 24+x, 22+y, 255);
            lcd.line(24-xp, 22-yp, 24+x, 22+y, 255);
        }

        // print the heading under the dial
        lcd.setfont(1);
        int xp, yp;
        xp = -11.0*sin(wind_rad), yp = 10.0*cos(wind_rad);
        static float nxp = 0, nyp = 0;
        nxp = (xp + 31*nxp)/32;
        nyp = (yp + 31*nyp)/32;
        xp = 24+nxp-12, yp = 22+nyp-8; 
        lcd.rectangle(xp-1, yp+3, xp+22, yp+14, 0); // clear heading text area
        lcd.setpos(xp, yp);
        lcd.print(status_buf[0]);
    }
    
    lcd.refresh();
#endif
}

void draw_barometer_graph()
{
#ifdef LCD
    static uint16_t baro_history[84];
    static uint8_t history_pos;
    static uint32_t baro_val, baro_count;
    static uint32_t last_baro_updatetime;
    if(!lcd_update)
        return;

    uint16_t time = millis();
    uint16_t dt = time - last_lcd_updatetime;
    if(dt < 100) // don't update faster
        return;

    baro_val += pressure_comp;
    baro_count++;

    last_lcd_updatetime = time;
    
    lcd_update = 0;

    dt = time - last_lcd_texttime;
    if(dt > 700) { // don't update text so fast
        int a, r;
        last_lcd_texttime = time;

        lcd.rectangle(0, 0, 10, 83, 0); // clear text

#if 1
        lcd.setfont(4);
        lcd.setpos(0, 0);
        a = pressure_comp/1e2, r = pressure_comp - a*1e2;
        a = snprintf(status_buf[2], sizeof status_buf[2], "%d", a);
        lcd.print(status_buf[2]);
        lcd.rectangle(0, a*7+3, 0, a*7+4, 255); // draw decimal
        snprintf(status_buf[2], sizeof status_buf[2], "%02d", r);
        lcd.setpos(0, a*7+6);
        lcd.print(status_buf[2]);
#endif
        
        a = temperature_comp / 100;
//        r = temperature_comp - a*100;
        snprintf(status_buf[3], sizeof status_buf[3], "%dC", a);
        
        lcd.setfont(4);
        lcd.setpos(0, 60);
        lcd.print(status_buf[3]);
    }

    uint32_t dt32 = millis() - last_baro_updatetime;
    
    if(dt32 > 60000) { // once a minute
        last_baro_updatetime += 60000;
        int32_t val = baro_val / baro_count;
        baro_val = baro_count = 0;
    
        int16_t bar = val - 80000;

        baro_history[history_pos++] = bar;
        if(history_pos == 84)
            history_pos = 0;

        lcd.rectangle(10, 0, 48, 84, 0); // clear graph

        for(int i=0; i<84; i++) {
            int p = (history_pos + i)%84;
            int v = baro_history[p] - bar;
            v /= 5;

            v += 28;
            if(v > 10)
                lcd.putpixel(v, i, 255);
        }
    }

    lcd.refresh();
#endif
}

void loop()
{
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    sleep_cpu();

    wdt_reset();

    read_pressure_temperature();

#ifdef LCD
    read_light();
#endif
#ifdef ANEMOMETER
    read_anemometer();
    draw_anemometer();
#else
    draw_barometer_graph();
#endif
}
