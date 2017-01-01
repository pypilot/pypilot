/* Copyright (C) 2016 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

/* read voltage on adc1 pin and output to usb

Run on linux to cat serial port:
 stty -F /dev/ttyUSB0 38400 ignbrk -icrnl -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke min 1 time 5 */

void setup() 
{ 
    Serial.begin(38400);
    pinMode(A0, OUTPUT);
    digitalWrite(A0, 0);    
    pinMode(A2, OUTPUT);      
    digitalWrite(A2, 1);
}

int ncount = 750;
int32_t read_adc(int channel)
{
    int32_t t = 0;
    for(int i=0; i<ncount; i++)
        t += analogRead(channel);

    return t;
}

#include <stdarg.h>
void p(char *fmt, ... ){
        char buf[128]; // resulting string limited to 128 chars
        va_list args;
        va_start (args, fmt );
        vsnprintf(buf, 128, fmt, args);
        va_end (args);
        Serial.print(buf);
}

void loop()
{
   int32_t dir  = (int32_t)read_adc(A1)/256; // 0-3000
   int32_t speed = 0;
//    char    c = v < 0 ? '-' : ' ';
//    p("{\"name\": \"windsensor\", \"value\": \"%c%ld.%03ld\"}\n", c, abs(v)/1000, abs(v)%1000);
   char buf[128];
   snprintf(buf, sizeof buf, "APMWV,%d.%d,R,%d.%d,K", (int)dir/10, (int)dir%10, (int)speed/10, (int)speed%10);
   
   uint8_t cksum = 0;
   for(int i=0; i<strlen(buf); i++)
       cksum ^= buf[i];
   
   char buf2[128];
   snprintf(buf2, sizeof buf2, "$%s*%02x\r\n", buf, cksum);
   Serial.print(buf2);
}
