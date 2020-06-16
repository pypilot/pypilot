/* Copyright (C) 2020 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

/* this program reads rf on pin2, and ir on pin3 and outputs
   received remote presses as a spi slave
*/

#include <Arduino.h>
#include <stdint.h>
#include <HardwareSerial.h>

#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/boot.h>
#include <util/delay.h>

#include "crc.h"

// input packet types
enum {SET_BACKLIGHT=0xa1};

// output packet types
enum {RF=0xa1, IR=0x06, AR=0x9c};

#include <RCSwitch.h>

#define DATA_PIN 2
#define DIR_PIN 4

#define LED_PIN 8

RCSwitch rf = RCSwitch();

#include <IRLibRecvPCI.h> 
#include <IRLibDecodeBase.h>
#include <IRLib_P01_NEC.h>    //Lowest numbered protocol 1st
#include <IRLib_P02_Sony.h>   // Include only protocols you want
#include <IRLib_P03_RC5.h>
#include <IRLib_P04_RC6.h>
#include <IRLib_P05_Panasonic_Old.h>
#include <IRLib_P07_NECx.h>
#include <IRLib_HashRaw.h>    //We need this for IRsendRaw
#include <IRLibCombo.h>
IRdecode myDecoder;   //create decoder
IRrecvPCI ir(3);//pin number for the receiver

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
  pinMode(3, INPUT);
  pinMode(DIR_PIN, INPUT);
  
  rf.enableReceive(0);  // Receiver on interrupt 0 => that is pin #2
  ir.enableIRIn();

  digitalWrite(LED_PIN, LOW); /* enable internal pullups */
  pinMode(LED_PIN, OUTPUT);
  
  // turn backlight on
  pinMode(A0, INPUT);

  digitalWrite(9, LOW); /* enable internal pullups */
  pinMode(9, OUTPUT);
#if 1
  TCNT1 = 0x1fff;
  //Configure TIMER1
        TCCR1A=_BV(COM1A1)|_BV(WGM11);        //NON Inverted PWM
        TCCR1B=_BV(WGM13)|_BV(WGM12)|_BV(CS11); //PRESCALER=8 MODE 14(FAST PWM)
        ICR1=1000;
        TIMSK1 = 0;
        OCR1A = 200;
#endif
}

static uint32_t lvalue, ltime;
static uint8_t count, lsource;

void send_code(uint8_t source, uint32_t value)
{
    if(source == lsource && value == lvalue)
        count++;
    else {
        // send code up for last key if key changed
        if(lvalue) {
            uint8_t *plvalue = (uint8_t*)&lvalue;
            uint8_t d[PKTSZ] = {lsource, plvalue[0], plvalue[1], plvalue[2], 0};
            spiout.push_packet(d);
        }
        count = 1;
    }
            
    uint8_t *pvalue = (uint8_t*)&value;
    uint8_t d[PKTSZ] = {source, pvalue[0], pvalue[1], pvalue[2], count};
    spiout.push_packet(d);
    lvalue = value;
    lsource = source;
    ltime = millis();
    digitalWrite(LED_PIN, HIGH); // turn on led to indicate remote received
}

void loop() {
    // parse incomming data
    uint8_t d[PKTSZ];
    if(spiin.pop_packet(d)) {
        if(d[0] == SET_BACKLIGHT) {
            // turn on backlight
            backlight_value = d[4];
	    OCR1A = backlight_value*10;
        }
    }


    // send code up message on timeout
    if(lvalue && millis() - ltime > 250) {
        uint8_t *plvalue = (uint8_t*)&lvalue;
        uint8_t d[PKTSZ] = {lsource, plvalue[0], plvalue[1], plvalue[2], 0};
        spiout.push_packet(d);
        lvalue = 0;
        digitalWrite(LED_PIN, LOW);
    }

    // read from IR??
    if (ir.getResults()) {
        myDecoder.decode();
        send_code(IR, (myDecoder.value<<8) | myDecoder.protocolNum);
        ir.enableIRIn();      //Restart receiver
    }

    if (rf.available()) {
        uint32_t value = rf.getReceivedValue();
        if(value && rf.getReceivedBitlength() == 24)
            send_code(RF, value);

        rf.resetAvailable();
    }
}
