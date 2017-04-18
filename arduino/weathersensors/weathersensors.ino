/* Copyright (C) 2016 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

/* this program interfaces with the bmp280 pressure sensor and outputs
 calibrated signalk data over serial at 115200 baud
 */
#include <Arduino.h>
#include <stdint.h>
#include <HardwareSerial.h>
#include <Wire.h>

const int analogInPin = A7;  // Analog input pin that the potentiometer is attached to

uint8_t have_bmp280 = 0;
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

void bmp280_setup()
{
  Wire.begin();        // join i2c bus (address optional for master)

#if 0
  Wire.beginTransmission(0x76);
  Wire.write(0xE0);
  Wire.write(0xB6);
  Wire.endTransmission();
#endif
  
  Wire.beginTransmission(0x76);
  Wire.write(0xD0);
  Wire.endTransmission();
  
  Wire.requestFrom(0x76, 1);
  if(Wire.read() != 0x58)
      return;

  have_bmp280 = 1;
  
  Wire.beginTransmission(0x76);
  Wire.write(0x88);
  Wire.endTransmission();
  
  Wire.requestFrom(0x76, 24);    // request 6 bytes from slave device #2
  dig_T1 = Wire.read() | Wire.read() << 8;
  dig_T2 = Wire.read() | Wire.read() << 8;
  dig_T3 = Wire.read() | Wire.read() << 8;
  dig_P1 = Wire.read() | Wire.read() << 8;
  dig_P2 = Wire.read() | Wire.read() << 8;
  dig_P3 = Wire.read() | Wire.read() << 8;
  dig_P4 = Wire.read() | Wire.read() << 8;
  dig_P5 = Wire.read() | Wire.read() << 8;
  dig_P6 = Wire.read() | Wire.read() << 8;
  dig_P7 = Wire.read() | Wire.read() << 8;
  dig_P8 = Wire.read() | Wire.read() << 8;
  dig_P9 = Wire.read() | Wire.read() << 8;
  
#if 0
  Serial.println("pressure compensation:");
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

  Wire.beginTransmission(0x76);
  Wire.write(0xF4);
  // b00011111  // configure
  Wire.write(0xff);
  Wire.endTransmission();
}

volatile unsigned int rotation_count;
volatile unsigned long lastt;
volatile unsigned long lastperiod;
void isr_anenometer_count()
{
  unsigned long t  = millis();
  if(t - lastt > 15){
    if(rotation_count == 0)
        lastperiod = 0;
    lastperiod += t - lastt;
    rotation_count++;
    lastt = t;
  }
}

 void setup()
{
  Serial.begin(38400);  // start serial for output
  bmp280_setup();

  attachInterrupt(0, isr_anenometer_count, FALLING);
  pinMode(analogInPin, INPUT);
  pinMode(2, INPUT_PULLUP);

  analogRead(analogInPin);
//  ADCSRA |= _BV(ADIE);
  ADCSRA |= _BV(ADSC);   // Set the Start Conversion flag.
}

static volatile uint32_t adcval;
static volatile uint16_t adccount;
ISR(ADC_vect)
{
    adcval += (int16_t)ADCW;
    ADCSRA |= _BV(ADSC);   // Set the Start Conversion flag.
    adccount++;
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



void read_anenometer() {
  static float lpdir, lp = .05;
  
  // read the analog in value:
  int sensorValue = 0;
#if 1
  if (!(ADCSRA & _BV(ADSC)))
      sensorValue = ADCW;

  ADCSRA |= _BV(ADSC);   // Set the Start Conversion flag.

//  sensorValue = analogRead(analogInPin);
#else
  cli();
  uint32_t adcval = adcval;
  uint16_t count = adccount;
  sei();
  sensorValue = adcval / count;
#endif

  float dir = sensorValue / 1024.0 * 360;

  if(lpdir - dir > 180)
      dir += 360;
  else if(dir - lpdir > 180)
      dir -= 360;
  
  lpdir = lp*dir + (1-lp)*lpdir;

  if(lpdir >= 360)
      lpdir -= 360;
  else if(lpdir < 0)
      lpdir += 360;

  // print the results to the serial monitor:
  static float knots;
  static int nowind;
  static unsigned long last_time;
  unsigned long time = millis();
  unsigned long dt = time - last_time;
  
  if(dt >= 100) { // output every 100ms
    last_time = time;
    int count;
    cli();
    count = rotation_count;
    long period = lastperiod;
    sei();
    if(count > 0) {
        rotation_count = 0;
        nowind = 0;
        knots = .868976 * 2.25 * 1000 * count / period;
    } else
        nowind++;

    if(nowind == 10)
      knots /= 2, nowind = 0;

    char buf[128];
    snprintf(buf, sizeof buf, "ARMWV,%d.%d,R,%d.%d,N,A", (int)lpdir, (int)(lpdir*10)%10, \
             (int)knots, (int)(knots*10)%10);
    send_nmea(buf);
  }
}


uint32_t pressure, temperature;
int bmp280_count;
void read_pressure_temperature()
{
    Wire.beginTransmission(0x76);
    Wire.write(0xF7);
    Wire.endTransmission();
  
    Wire.requestFrom(0x76, 6);    // request 6 bytes from slave device #2

    uint8_t buf[6] = {0};
    int i = 0;
    while(Wire.available())    // slave may send less than requested
    {
        buf[i++] = Wire.read();
        if(i == 6)
            break;
    }     
    
    int32_t p, t;
    
    p = (int32_t)buf[0] << 16 | (int32_t)buf[1] << 8 | (int32_t)buf[2];
    t = (int32_t)buf[3] << 16 | (int32_t)buf[4] << 8 | (int32_t)buf[5];
    
    if(t == 0 || p == 0) {
        Serial.println("reset bmp280");
        bmp280_setup();
        //  delay_ms(100);
        return;
    }

    pressure += p >> 2;
    temperature += t >> 2;
    bmp280_count++;
   
    if(bmp280_count == 1024) {
        bmp280_count = 0;
        pressure >>= 12;
        temperature >>= 12;
    
        int32_t temperature_comp = bmp280_compensate_T_int32(temperature);
        int32_t pressure_comp = bmp280_compensate_P_int64(pressure) >> 8;
  
        char buf[128];
        int ap = pressure_comp/100000;
        uint32_t rp = pressure_comp - ap*100000UL;
        snprintf(buf, sizeof buf, "ARMDA,,,%d.%05ld,B,,,,,,,,,,,,,,,,", ap, rp);
        send_nmea(buf);

        int a = temperature_comp / 100;
        int r = temperature_comp - a*100;
        snprintf(buf, sizeof buf, "ARMTA,%d.%02d,C", a, r);
        send_nmea(buf);

        pressure = temperature = 0;
    }
}

void loop()
{
    if(have_bmp280)
        read_pressure_temperature();
    read_anenometer();
}
