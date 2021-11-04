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

uint8_t flag;

// do nothing, just wake up
//ISR(PCINT2_vect) __attribute__((naked));
ISR(PCINT2_vect) {
//    asm volatile ("reti");
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
static uint32_t code, timeout;
void loop() {
    // test keys
    uint32_t t0 = millis();
    uint8_t pind = PIND;
    if(pind != 0xff) {
        LED2_ON;
        BOOST_ON;
        static uint8_t wd;
        if(flag) {
            wd = !wd;
            flag = 0;
        }

        // construct output using PC0-PC3 to give unique codes for different remote types
        uint32_t pinc = (PINC & 0xf) ^ 0xc;
        uint32_t keys = (pinc << 16) | (0xff00 & (~pind << 8)) | pind;
        if(wd)
            code = 0xa00000 | keys;
        else
            code = 0xd00000 | keys;
        timeout = t0;
    }

    if(code) {
        if(t0 - timeout < 200UL) {
            rf.send(code, 24);
        } else if(t0 - timeout < 400UL) {
            LED1_ON;
            LED2_OFF;
            rf.send(0x7c2933UL, 24); // release code
        } else {
            LED1_OFF;
            LED2_OFF;
            code = 0;
            BOOST_OFF;
        }
    } else {
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
