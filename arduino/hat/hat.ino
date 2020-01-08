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

#include "crc.h"

//#define DEBUG_SERIAL

enum {SET_BACKLIGHT=0xa1};
enum {RF=0xa1, IR=0x06, AR=0x9c};

#include <RCSwitch.h>

#define DATA_PIN 2
#define DIR_PIN 3
#define PWR_PIN 4

RCSwitch rf = RCSwitch();
uint8_t backlight_value = 128; // determines when backlight turns on

#define PKTSZ 5

#define RB_SIZE 48
struct ringbuffer {
    uint8_t d[RB_SIZE];
    uint8_t head, tail;

    ringbuffer() { head = tail = 0; }

    void push_packet(uint8_t d[PKTSZ]) {
        for(int i=0; i<5; i++) {
            cli();
            push(d[i]);
            sei();
        }
        cli();
        push(crc8(d, 5));
        sei();
    }

    void push(uint8_t v) {
        d[head++] = v;
        if(head >= RB_SIZE)
            head = 0;
    }

    bool empty() {
        return tail == head;
    }

    uint8_t pop() {
        uint8_t r = d[tail++];
        if(tail >= RB_SIZE)
            tail = 0;
        return r;
    }

} spiout;

struct recvpacketbuffer : public ringbuffer
{
    uint8_t n;
    uint8_t d[PKTSZ+1];
    bool pop_packet(uint8_t e[PKTSZ]) {
        while(!empty()) {
            cli();
            d[n] = pop();
            sei();
            if(n >= PKTSZ) {
                if(crc8(d, PKTSZ) == d[PKTSZ]) {
                    memcpy(e, d, PKTSZ);
                    n = 0;
                    return true;
                }
                memmove(d, d+1, PKTSZ);
            } else
                n++;
        }
        return false;
    }
} spiin;

// SPI interrupt routine
ISR (SPI_STC_vect)
{
    spiin.push(SPDR);
    if(spiout.empty())
        SPDR = 0;
    else
        SPDR = spiout.pop();
}

void setup() {
// turn on SPI in slave mode
    SPCR |= _BV(SPE);

    // turn on interrupts
    SPCR |= _BV(SPIE);
    pinMode(MISO, OUTPUT);

#ifdef DEBUG_SERIAL
    Serial.begin(38400);
      Serial.print("Begin");
#endif

  pinMode(DATA_PIN, INPUT);
  pinMode(DIR_PIN, INPUT);
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  
  rf.enableReceive(0);  // Receiver on interrupt 0 => that is pin #2

  // turn backlight on
  pinMode(A0, OUTPUT);
  digitalWrite(A0, HIGH);
}

void loop() {
    // parse incomming data
    uint8_t d[PKTSZ];
    if(spiin.pop_packet(d)) {
        if(d[0] == SET_BACKLIGHT) {
            // turn on backlight
            backlight_value = d[1];
        }
    }


    static uint32_t lvalue, ltime;
    static uint8_t count;
    uint8_t *plvalue = (uint8_t*)&lvalue;
    // send code up message on timeout
    if(lvalue && millis() - ltime > 100) {
        uint8_t d[PKTSZ] = {RF, plvalue[0], plvalue[1], plvalue[2], 0};
        spiout.push_packet(d);
        lvalue = 0;
    }
    
    if (!rf.available())
        return;
    
    uint32_t rvalue = rf.getReceivedValue();
    uint8_t *pvalue = (uint8_t*)&rvalue;

    
#ifdef DEBUG_SERIAL
    if (rvalue == 0) {
        Serial.print("Unknown encoding");
    } else {
        Serial.print("Received ");
        Serial.print( rf.getReceivedValue() );
        Serial.print(" / ");
        Serial.print( rf.getReceivedBitlength() );
        Serial.print("bit ");
        Serial.print("Protocol: ");
        Serial.println( rf.getReceivedProtocol() );
        unsigned int *raw = rf.getReceivedRawdata();
        Serial.print("timings: ");
        for(int i=0; i<16; i++) {
            Serial.print(raw[i]);
            Serial.print(" ");
        }
        Serial.println("");
    }
#endif

    if(rvalue && rf.getReceivedBitlength() == 24) {
        if(rvalue == lvalue)
            count++;
        else {
            // send code up for last key if key changed
            if(lvalue) {
                uint8_t d[PKTSZ] = {RF, plvalue[0], plvalue[1], plvalue[2], 0};
                spiout.push_packet(d);
            }
            count = 1;
        }
            
        uint8_t d[PKTSZ] = {RF, pvalue[0], pvalue[1], pvalue[2], count};
        spiout.push_packet(d);
        lvalue = rvalue;
        ltime = millis();
    }

    rf.resetAvailable();
}
