/* Copyright (C) 2023 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

#include <RCSwitch.h>

RCSwitch rf = RCSwitch();

#include <avr/sleep.h>
#include <avr/power.h>

#define LED1 PB2
#define LED2 PB5
#define BOOST PB0

#define LED1_ON  PORTB |= _BV(LED1)
#define LED1_OFF PORTB &= ~_BV(LED1)

#define LED2_ON  PORTB |= _BV(LED2)
#define LED2_OFF PORTB &= ~_BV(LED2)

// supply 12 volts for transmitter
#define BOOST_ON  PORTB |= _BV(BOOST)
#define BOOST_OFF PORTB &= ~_BV(BOOST)

#define MIN_PACKETS 3

uint8_t flag;

ISR(PCINT2_vect) {
    if(PIND==0xff)
        flag = 1; // toggle flag
}

// the setup routine runs once when you press reset:
void setup() {
    PCICR |= _BV(PCIE2); // enable pin change interrupt
    PCMSK2 = 0xff; // port D used for 8 keys, wake on any change

    ADCSRA=0;

    cli();
    CLKPR = _BV(CLKPCE);
    CLKPR = _BV(CLKPS1); // divide by 4 (2mhz)
    sei();

    PORTD = 0xff; // internal pullup resistors for all port D
    rf.enableTransmit(9); // d9
    rf.setRepeatTransmit(1);

    // these appear not to affect sleep consumption...
    DIDR0 = 0x30; // disable digital io on analog pins 4 and 5
    PORTC = 0xff; // enable pullup
    DDRC = 0; // all input
    PORTB = _BV(PB3) | _BV(PB4);
  
    // for led
    DDRB|=_BV(LED1) | _BV(LED2) | _BV(BOOST);

    PORTC = PINC; // enable pullups only on pins without pulldown straps
}

// the loop routine runs over and over again forever:
static uint8_t key_count;

uint8_t count_bits (uint8_t byte)
{
    static const uint8_t nl[16] =  {4, 3, 3, 2, 3, 2, 2, 1, 3, 2, 2, 1, 2, 1, 1, 0};
    return nl[byte >> 4] + nl[byte & 0xf];
}

struct event {
    uint32_t code;
    uint8_t sent, released;
} events[10];

#define MAX_EVENTS (int)((sizeof events) / (sizeof *events))
static uint8_t event_head, event_tail;

event *event_get()
{
    if(event_head == event_tail)
        return NULL;

    return events + event_tail;
}

void loop() {
    // test keys
    //uint32_t t0 = millis();
    uint8_t pind = PIND;
    if(pind != 0xff) {
        static uint8_t wd;
        if(flag) {
            wd = !wd;
            flag = 0;
        }

        uint8_t count = count_bits(pind);
        if(count <= 2 && count > key_count) {
            // construct output using PC0-PC3 to give unique codes for different remote types
            uint32_t pinc = (PINC & 0xf) ^ 0xc;
            uint32_t keys = (pinc << 16) | (0xff00 & (~pind << 8)) | pind;
            uint32_t code;
            if(wd)
                code = 0xa00000 | keys;
            else
                code = 0xd00000 | keys;
            key_count = count;

            event *evt = event_get();
            if(evt && !evt->released) {
                evt->code = code;
                evt->sent = 0;
            } else {
                int count = event_head - event_tail;
                if(count < 0)
                    count += MAX_EVENTS;
                if(count < MAX_EVENTS - 1) {
                    evt = events + event_head;
    
                    evt->code = code;
                    evt->sent = 0;
                    evt->released = 0;

                    event_head++;
                    if(event_head >= MAX_EVENTS)
                        event_head = 0;
                }
            }
        }
    } else {
        event *evt = event_get();
        if(evt)
            evt->released = 1;
        key_count = 0;
    }

    // process events
    event *evt = event_get();
    if(evt) {
        BOOST_ON;
        if(evt->code) {
            LED1_OFF;
            LED2_ON;
            rf.send(evt->code, 24);
            evt->sent++;
            if(evt->released && evt->sent >= MIN_PACKETS) {
                evt->code = 0;
                evt->sent = 0;
            }
        } else {
            LED1_ON;
            LED2_OFF;
            rf.send(0x7c2933UL, 24); // release code
            evt->sent++;

            if(evt->sent >= MIN_PACKETS) {
                event_tail++;
                if(event_tail >= MAX_EVENTS)
                    event_tail = 0;
            }
        }
    } else {
        LED1_OFF;
        LED2_OFF;
        BOOST_OFF;
        
        // if all settings are correct (no BOD), deep sleep is 4uA
        // wrong settings will drain battery
        //    power_all_disable (); // needed?
        
        //DIDR0 = 0x3f; // disable digital io on all
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_enable();
        sleep_bod_disable();
        sleep_cpu();
        delay(1);
        //DIDR0 = 0x30; // disable digital io on analog pins 4 and 5
    }
}
