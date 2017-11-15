/* Copyright (C) 2017 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

/* this program interfaces with the bmp280 pressure sensor and outputs
 calibrated signalk data over serial at 38400 baud
 */

/* anenometer wires

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

extern "C" {
  #include <twi.h>
}

#define LCD
#ifdef LCD
static PCD8544 lcd(13, 11, 8, 7, 99);
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
      d[0] = 0xe0;
      d[1] = 0xb6;
      twi_writeTo(0x76, d, 2, 1, 1);
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
void isr_anenometer_count()
{
    static uint16_t lastt;
    int t = millis();
    uint16_t period = t-lastt;
    if(period < 15) // debounce, at least for less than 130 knots of wind
        return;

    lastt = t;
    lastperiod += period;
    rotation_count++;
}

void setup()
{
  Serial.begin(38400);  // start serial for output
  bmX280_setup();

  attachInterrupt(0, isr_anenometer_count, FALLING);
  pinMode(analogInPin, INPUT);
  pinMode( analogLightPin, INPUT);
  pinMode(2, INPUT_PULLUP);

  //analogRead(analogInPin); // select channel
  ADMUX = _BV(REFS0) | 6;
  ADCSRA |= _BV(ADIE);
  ADCSRA |= _BV(ADSC);   // Set the Start Conversion flag.

#ifdef LCD
    // PCD8544-compatible displays may have a different resolution...
  lcd.begin(84, 48);
#endif
}

static volatile uint8_t adcchannel;
static volatile uint32_t adcval[2];
static volatile int16_t adccount[2];

ISR(ADC_vect)
{
    adccount[adcchannel]++;
    if(adccount[adcchannel] > 0)
        adcval[adcchannel] += (int16_t)ADCW;
    if(adcchannel == 0 && adccount[adcchannel] == 1)
        ADMUX = _BV(REFS0) | 7, adcchannel = 1;
    ADCSRA |= _BV(ADSC);   // Set the Start Conversion flag.
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
float wind_dir, wind_speed;
void read_anenometer()
{
    static float lpdir;
    const float lp = .1;
//    lp = 1;
    // read the analog in value:
    float sensorValue = 0;
#if 0
    if (!(ADCSRA & _BV(ADSC)))
        sensorValue = ADCW;
    
    ADCSRA |= _BV(ADSC);   // Set the Start Conversion flag.
    
//  sensorValue = analogRead(analogInPin);
#else
    cli();
    int16_t light = adcval[0];
    uint32_t val = adcval[1];
    uint16_t count = adccount[1];
    // reset the adc
    adcval[0] = adcval[1] = 0;
    adccount[0] = adccount[1] = -4; // skip the first 4 readings after changing channel
    adcchannel = 0;
    ADMUX = _BV(REFS0) | 6;
    sei();
    sensorValue = val / count;

    // filter and map light value
    static int lastlight, lighton;
    light = (light + 31*lastlight) / 32;
    lastlight = light;

    int pwm = 255L*light / 600 - 120;
    if(!lighton) {
        if(pwm < 240)
            lighton = 1;
        else
            pwm = 255;
    }
    if(pwm < 160) // limit backlight brightness
        pwm = 160;
    if(pwm > 255) 
        pwm = 255, lighton = 0;
    #if 0
    Serial.print("light ");
    Serial.print(light);
    Serial.print(" pwm ");
    Serial.print(pwm);
    Serial.println("");
    #endif
    analogWrite(analogBacklightPin, pwm);
#endif
    
//  float dir = sensorValue / 1024.0 * 360;
    // compensate 13 degree deadband in potentiometer
    float dir = (sensorValue + 13) * .34;
    
    if(lpdir - dir > 180)
        dir += 360;
    else if(dir - lpdir > 180)
        dir -= 360;
    
    lpdir = lp*dir + (1-lp)*lpdir;
//    lpdir = dir;
    
    if(lpdir >= 360)
        lpdir -= 360;
    else if(lpdir < 0)
        lpdir += 360;

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
        static float knots = 0;
        const int nowindtimeout = 30;
        if(count) {
            if(nowindcount!=nowindtimeout)
                knots = .868976 * 2.25 * 1000 * count / period;
            nowindcount = 0;
        } else {
            if(nowindcount<nowindtimeout)
                nowindcount++;
            else
                knots = 0;
        }

        char buf[128];
        snprintf(buf, sizeof buf, "ARMWV,%d.%02d,R,%d.%02d,N,A", (int)lpdir, (uint16_t)(lpdir*100.0)%100U, (int)knots, (int)(knots*100)%100);
        //        Serial.println(lpdir*100);
        //        Serial.println((int)(uint16_t)(lpdir*100.0)%100U);

        send_nmea(buf);

        wind_dir = lpdir;
        wind_speed = knots;
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
   
    if(bmp280_count == 1024) {
        bmp280_count = 0;
        pressure >>= 12;
        temperature >>= 12;
    
        temperature_comp = bmp280_compensate_T_int32(temperature);
        pressure_comp = bmp280_compensate_P_int64(pressure) >> 8;
        pressure = temperature = 0;

        if(!have_bmp280) {
            /* only re-run setup when count elapse */
            bmX280_setup();
            return;
        }
  
        char buf[128];
        int ap = pressure_comp/100000;
        uint32_t rp = pressure_comp - ap*100000UL;
        snprintf(buf, sizeof buf, "ARMDA,,,%d.%05ld,B,,,,,,,,,,,,,,,,", ap, rp);
        send_nmea(buf);

        int a = temperature_comp / 100;
        int r = temperature_comp - a*100;
        snprintf(buf, sizeof buf, "ARMTA,%d.%02d,C", a, r);
        send_nmea(buf);

        lcd_update  = 1;
    }
}

void draw()
{
#ifdef LCD
    if(!lcd_update)
        return;

    uint16_t time = millis();
    static uint16_t last_lcd_updatetime, last_lcd_texttime;
    uint16_t dt = time - last_lcd_updatetime;
    if(dt < 100) // don't update faster than 2frames per second
        return;

    last_lcd_updatetime = time;
    
    lcd_update = 0;
    lcd.clear();

    static char status_buf[4][16];

    dt = time - last_lcd_texttime;
    if(dt > 700) { // don't update text so fast
        last_lcd_texttime = time;
        int a = temperature_comp / 100;
        int r = temperature_comp - a*100;
        snprintf(status_buf[3], sizeof status_buf[3], "%d.%02dC", a, r);
        a = pressure_comp/1e2, r = pressure_comp - a*1e2;
        snprintf(status_buf[2], sizeof status_buf[2], "%d.%02d", a, r);

        snprintf(status_buf[0], sizeof status_buf[0], "%02d", (int) round(wind_dir));
        snprintf(status_buf[1], sizeof status_buf[1], "%02d", (int) round(wind_speed));
    }

    int page = 1;
    switch(page) {
    case 0:
        
        lcd.setfont(1);
        lcd.setpos(0, 0);
        lcd.print(status_buf[0]);
        lcd.setpos(0, 25);
        lcd.print(status_buf[1]);
        lcd.rectangle(30, 49, 32, 51, 255); // decimal point

        lcd.setfont(0);
        lcd.setpos(0, 56);
        lcd.print(status_buf[2]);
        lcd.setpos(0, 70);
        lcd.print(status_buf[3]);
        break;
    case 1:
        // draw direction dial for wind
        lcd.circle(24, 24, 22, 255);
        float wind_rad = wind_dir/180*M_PI;
        {
            int r = 22;
            int x = r*sin(wind_rad);
            int y = -r*cos(wind_rad);
            for(int s=1; s<3; s++) {
                int xp = s*cos(wind_rad);
                int yp = s*sin(wind_rad);
                lcd.line(24+xp, 24+yp, 24+x, 24+y, 255);
                lcd.line(24-xp, 24-yp, 24+x, 24+y, 255);
            }
        }

        // draw wind direction behind arrow
        lcd.setfont(1);
        int xp = -10*sin(wind_rad);
        int yp = 10*cos(wind_rad);
        lcd.setpos(24+xp-12, 24+yp-14);
        lcd.print(status_buf[1]);

        // print the heading under the dial
        lcd.setfont(1);
        lcd.setpos(0, 42);
        lcd.print(status_buf[0]);

        lcd.setfont(0);
        lcd.setpos(0, 61);
        lcd.print(status_buf[2]);

        lcd.setpos(0, 71);
        lcd.print(status_buf[3]);
        break;
    }
    uint16_t time1 = millis();

    

  lcd.refresh();
#endif
}

void loop()
{
      set_sleep_mode(SLEEP_MODE_IDLE);
    read_pressure_temperature();
    read_anenometer();

    draw();
}
