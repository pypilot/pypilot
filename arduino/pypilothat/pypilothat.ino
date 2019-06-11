/* Copyright (C) 2019 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

/* this program reads rf key codes from the spi port, and applies
   commands as configured by the webapp on port 33333
*/

#include <Arduino.h>
#include <stdint.h>
#include <HardwareSerial.h>

#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/boot.h>
#include <util/delay.h>

//#define DEBUG_SERIAL

#define SET_BACKLIGHT 0xe8

#include <RCSwitch.h>

#define DATA_PIN 2
#define DIR_PIN 3
#define PWR_PIN 4

RCSwitch mySwitch = RCSwitch();
uint8_t spiread[4];
uint8_t backlight_value = 128; // determines when backlight turns on

volatile static uint32_t spiread_rfkeys, spiread_rfkeys_buf;
// SPI interrupt routine
ISR (SPI_STC_vect)
{
    uint8_t *d = (uint8_t*)&spiread_rfkeys_buf;
    static uint8_t spipos, spiposin;
    static uint8_t spiin[4];
    uint8_t c = SPDR;

    spiin[spiposin] = c;
    if(c == 0 && spiposin < 2)
        spiposin = 0;
    else
        spiposin++;

    if (spiposin == 4) {
        memcpy(spiread, spiin, sizeof spiin);
        spiposin = 0;
    }
        
    SPDR = d[3-spipos++];

    if(spipos >= 4) {
        spipos = 0;
        spiread_rfkeys_buf = spiread_rfkeys;
        spiread_rfkeys = 0;
    }
}

void setup() {
// turn on SPI in slave mode
    SPCR |= _BV(SPE);

    // turn on interrupts
    SPCR |= _BV(SPIE);
    pinMode(MISO, OUTPUT);

    // turn on backlight
    pinMode(A0, OUTPUT);
    digitalWrite(A0, HIGH);

#ifdef DEBUG_SERIAL
    Serial.begin(38400);
      Serial.print("Begin");
#endif

  pinMode(DATA_PIN, INPUT);
  pinMode(DIR_PIN, INPUT);
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  
  mySwitch.enableReceive(0);  // Receiver on interrupt 0 => that is pin #2
}

void loop() {
    // valid packet is 0 command value xor
    uint8_t command = spiread[1], value = spiread[2], xorv = spiread[3];
    if(command) {
        if((command ^ value) == xorv) {
            if(command == SET_BACKLIGHT) {
                backlight_value = value;
            }
        }

    }
    
  if (!mySwitch.available())
      return;
    
  int rvalue = mySwitch.getReceivedValue();
    
#ifdef DEBUG_SERIAL
  if (rvalue == 0) {
      Serial.print("Unknown encoding");
  } else {
      Serial.print("Received ");
      Serial.print( mySwitch.getReceivedValue() );
      Serial.print(" / ");
      Serial.print( mySwitch.getReceivedBitlength() );
      Serial.print("bit ");
      Serial.print("Protocol: ");
      Serial.println( mySwitch.getReceivedProtocol() );
      unsigned int *raw = mySwitch.getReceivedRawdata();
      Serial.print("timings: ");
      for(int i=0; i<16; i++) {
          Serial.print(raw[i]);
          Serial.print(" ");
      }
      Serial.println("");
  }
#endif

  if(rvalue && mySwitch.getReceivedBitlength() == 24)
      spiread_rfkeys = rvalue;
  mySwitch.resetAvailable();
  
}
