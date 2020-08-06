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
//#include <HardwareSerial.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/boot.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#include <RCSwitch.h>

#define DATA_PIN 2
#define DIR_PIN 4
#define LED_PIN 8

// start byte $ followed by PACKET_LEN bytes, and a parity byte
#define PACKET_LEN 6

// of packet bytes, first byte defines message type 
enum {RF=0x01, IR=0x02, GP=0x03, VOLTAGE=0x04, SET_BACKLIGHT=0x16, SET_BUZZER=0x17, SET_BAUD=0x18};

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
//IRrecvPCI ir(3);//pin number for the receiver

uint8_t backlight_value = 64; // determines when backlight turns on
uint8_t backlight_polarity = 0;
uint8_t backlight_value_ee EEMEM = 128; // determines when backlight turns on
uint8_t backlight_polarity_ee EEMEM = 0;
uint8_t serial_baud_ee EEMEM = 0;

#define RB_CREATE(NAME, SIZE)                                           \
    uint8_t rb_##NAME[SIZE];                                            \
    volatile uint8_t *rb_##NAME##_head=rb_##NAME, *rb_##NAME##_tail=rb_##NAME; \
    volatile uint8_t *rb_##NAME##_end = rb_##NAME + SIZE;

#define RB_PUT(NAME, SRC)                                 \
    {                                                     \
        *rb_##NAME##_head = SRC;                          \
        if(++rb_##NAME##_head >= rb_##NAME##_end)         \
            rb_##NAME##_head = rb_##NAME;                 \
    }

#define RB_GET(NAME, DEST)                                \
    {                                                     \
        if(rb_##NAME##_head == rb_##NAME##_tail)          \
            DEST = 0;                                     \
        else {                                            \
            DEST = *rb_##NAME##_tail;                     \
            if(++rb_##NAME##_tail == rb_##NAME##_end)     \
                rb_##NAME##_tail = rb_##NAME;             \
        }                                                 \
    } 

#define RB_COUNT(NAME)                                                  \
    (rb_##NAME##_head >= rb_##NAME##_tail ?                             \
     rb_##NAME##_head - rb_##NAME##_tail :                              \
     (rb_##NAME##_head - rb_##NAME + rb_##NAME##_end - rb_##NAME##_tail))

#define RB_EMPTY(NAME) (rb_##NAME##_head == rb_##NAME##_tail)

RB_CREATE(serial_out, 224)
RB_CREATE(serial_in, 224)
RB_CREATE(data_in, 128)

ISR(USART_RX_vect)
{
    sei(); // needed because spi runs faster
    RB_PUT(serial_out, UDR0);
}

ISR(USART_UDRE_vect)
{
    UCSR0B &= ~_BV(UDRIE0);
    sei();
    if(!RB_EMPTY(serial_in)) {
        RB_GET(serial_in, UDR0);
        UCSR0B |= _BV(UDRIE0);
    } else
        UCSR0B &= ~_BV(UDRIE0);
}

ISR (SPI_STC_vect) // SPI interrupt routine
{
    uint8_t c = SPDR;
    if(c>127)
        RB_PUT(data_in, c)
    else if(c) {
        UCSR0B |= _BV(UDRIE0);
        RB_PUT(serial_in, c);
    }
    RB_GET(serial_out, SPDR);
}
 
ISR(WDT_vect)
{
    wdt_reset();
    wdt_disable();

    asm volatile ("ijmp" ::"z" (0x0000)); // soft reset
}

void Serial_begin(uint8_t baud)
{

    UCSR0A = 0;//_BV(U2X0);
    
    // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
    // 0, 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600
    if(baud == 5) {
        // allow 4800
        UBRR0H = 0;
        UBRR0L = 103;
    } else {
        // otherwise 38400
        UBRR0H = 0;
        UBRR0L = 12;
    }
    
    UCSR0B = _BV(TXEN0) | _BV(RXEN0) | _BV(RXCIE0);
}

void setup() {
    cli();
    wdt_reset();
    // 250 millisecond
    wdt_disable();
    //WDTCSR = (1<<WDCE)|(1<<WDE);
//    WDTCSR = (1<<WDIE) | (1<<WDP2);
    WDTCSR = (1<<WDCE)|(1<<WDE);
    WDTCSR = 0;
    sei();

    backlight_value = eeprom_read_byte(&backlight_value_ee);
    backlight_polarity = eeprom_read_byte(&backlight_polarity_ee);
    uint8_t baud = eeprom_read_byte(&serial_baud_ee);
    Serial_begin(baud);

    // enable adc with 128 division
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
    ADCSRA |= _BV(ADSC);

    // turn on SPI in slave mode
    SPCR |= _BV(SPE);
    // turn on interrupts
    SPCR |= _BV(SPIE);
    pinMode(MISO, OUTPUT);

    // buzzer
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, INPUT_PULLUP);

    pinMode(DATA_PIN, INPUT);
    pinMode(3, INPUT);
    pinMode(DIR_PIN, INPUT);

    rf.enableReceive(0);  // Receiver on interrupt 0 => that is pin #2
//    ir.enableIRIn();

    digitalWrite(LED_PIN, LOW); /* enable internal pullups */
    pinMode(LED_PIN, OUTPUT);

    for(int i=0; i<6; i++)
        pinMode(A0+i, INPUT_PULLUP);

    digitalWrite(9, LOW); /* enable internal pullups */
    pinMode(9, OUTPUT);

    TCNT1 = 0x1fff;
    //Configure TIMER1 to drive backlight variable pwm
    TCCR1A=_BV(COM1A1)|_BV(WGM11);        //NON Inverted PWM
    TCCR1B=_BV(WGM13)|_BV(WGM12)|_BV(CS11); //PRESCALER=8 MODE 14(FAST PWM)
    ICR1 = 1000; // 1khz
    TIMSK1 = 0;
//    OCR1A = 100;
}

// interrupt driven pwm gives inverting signals (highest volume) with variable frequency
ISR(TIMER2_OVF_vect) __attribute__((naked));
ISR(TIMER2_OVF_vect)
{
    PORTD |= _BV(PD5);
    PORTD &= ~_BV(PD6);
    asm volatile ("reti");
}

ISR(TIMER2_COMPB_vect) __attribute__((naked));
ISR(TIMER2_COMPB_vect)
{
    PORTD |= _BV(PD6);
    PORTD &= ~_BV(PD5);
    asm volatile ("reti");
}

uint32_t buzzer_timeout;
void buzzer_on(uint8_t freq, uint8_t timeout)
{
    buzzer_timeout = millis() + timeout; // duration
    if(buzzer_timeout > 4294963200UL)
        buzzer_timeout = 1; // in case of 32 bit wrap.. needed???
        
    TCCR2A = _BV(WGM20) | _BV(WGM21);
    TCCR2B = _BV(WGM22) | _BV(CS21) | _BV(CS20); // divide by 32

    if(freq < 32)
        freq = 32;
    OCR2A = freq;
    OCR2B = freq/2;
    
    TIMSK2 |= _BV(OCIE2A) | _BV(TOIE2);
}

struct codes_type {
    uint32_t lvalue, ltime;
    uint8_t repeat_count;
};

static struct codes_type codes[4] = {0};
static uint16_t adc_avg[3], adc_count, adc_cycles=64;
static uint32_t adc_a;

void read_data()
{
    uint8_t data[PACKET_LEN];
    uint8_t start = 0;
    while(start != '$') {
        if(RB_COUNT(data_in) < PACKET_LEN + 2)
            return;
        RB_GET(data_in, start);
        start &= 0x7f;
    }
        
    uint8_t parity, p=0;
    for(uint8_t i=0; i<PACKET_LEN; i++) {
        RB_GET(data_in, data[i]);
        data[i] &= 0x7f;
        p ^= data[i];
    }

    RB_GET(data_in, parity);
    parity &= 0x7f;

    // ensure we don't get bad commands
    if(p != parity)
        return;

    uint8_t cmd = data[0], *d = data+1;
    switch(cmd) {
    case SET_BACKLIGHT:
        // turn on backlight
        if(backlight_value == d[0] && backlight_polarity == d[1])
            break;
        adc_cycles=64; // update timer from adc and ambient
        backlight_value = d[0];
        backlight_polarity = d[1];
        eeprom_write_byte(&backlight_value_ee, backlight_value);
        eeprom_write_byte(&backlight_polarity_ee, backlight_polarity);
        break;

    case SET_BUZZER:
        //buzzer_on(data[1], data[2]); // frequency
        break;

    case SET_BAUD:
    {
        eeprom_update_byte(&serial_baud_ee, d[0]);
        Serial_begin(d[0]);
    } break;
    }
}

void send(uint8_t id, uint8_t d[PACKET_LEN])
{
    uint8_t st = '$'|0x80;
    cli();
    RB_PUT(serial_out, st);
    sei();
    id |= 0x80;
    cli();
    RB_PUT(serial_out, id);
    sei();
    uint8_t parity = 0;
    for(int i = 0; i < PACKET_LEN; i++) {
        uint8_t v = d[i] | 0x80;
        parity ^= d[i];
        cli();
        RB_PUT(serial_out, v);
        sei();
    }
    cli();
    RB_PUT(serial_out, parity | 0x80);
    sei();
}

void send_code(uint8_t source, uint32_t value)
{
    // put all 24 bits into 4 7bit bytes.
    uint8_t *p1value = (uint8_t*)&value;
    uint32_t cvalue = value & 0x7f7f7f7f;
    uint8_t *pvalue = (uint8_t*)&cvalue;
    pvalue[3] = ((p1value[0]&0x80) >> 1) | ((p1value[1]&0x80) >> 2) | ((p1value[2]&0x80)) >> 3;
        
    if(cvalue == codes[source].lvalue) {
        if(++codes[source].repeat_count > 127)
            codes[source].repeat_count = 1;
    } else {
        // unfortunately IR has infrequent but soemtimes wrong value codes
        // so sending keyup from previous key if a new down is not used here.
        if(source == IR && codes[source].lvalue && value)
            return;
        // send code up for last key if key changed
        if(codes[source].lvalue) {
            uint8_t *plvalue = (uint8_t*)&codes[source].lvalue;
            uint8_t d[PACKET_LEN] = {plvalue[0], plvalue[1], plvalue[2], plvalue[3], 0, 0};
            send(source, d);
        }
        codes[source].repeat_count = 1;
    }

    if(cvalue) {
        uint8_t d[PACKET_LEN] = {pvalue[0], pvalue[1], pvalue[2], pvalue[3], codes[source].repeat_count, 0};
        send(source, d);
        codes[source].lvalue = cvalue;

        codes[source].ltime = millis();
        digitalWrite(LED_PIN, HIGH); // turn on led to indicate remote received
    }
}

void read_analog() {
    static uint8_t channel;
    const uint8_t channels[] = {_BV(MUX1) | _BV(MUX2),              // 5v through divider
                                _BV(MUX0) | _BV(MUX1) | _BV(MUX2),  // ambient light sensor
                                _BV(MUX1) | _BV(MUX2) | _BV(MUX3)}; // reference voltage
    if(ADCSRA & _BV(ADSC))
        return; // not ready yet

    adc_a += ADCW;
    ADCSRA |= _BV(ADSC);
    if(++adc_count < 256)
        return;

    adc_avg[channel] = adc_a >> 4;
    adc_count = 0;
    adc_a = 0;
    
    if(++channel == sizeof channels)
        channel = 0;

    ADMUX = _BV(REFS0) | channels[channel]; // select channel at 5 volts
    ADCSRA |= _BV(ADSC);

    if(++adc_cycles < 64)
        return;

    if(!RB_EMPTY(serial_out)) // don't report volts if other data is being sent
        return;

    adc_cycles = 0;
    uint16_t ambient = adc_avg[1];
    uint16_t reference = adc_avg[2];

    int ocr1a = backlight_value*10 - ambient/100;
    if(ocr1a < 0)
        ocr1a = 0;
    if(ocr1a > 1000)
        ocr1a = 1000;
    if(backlight_polarity)
        ocr1a = 1000 - ocr1a;
    OCR1A = ocr1a;

    // calculate input voltage (should be near 3.3)
    // reference/16/1023*Vin = 1.1
    // Vin = 1.1*1023*16*1000 / reference
    uint16_t vin = 18004800UL / reference; // in millivolts

    // calculate input power voltage through input divider
    // Vcc = 2*power/4/1023*vin
    uint16_t vcc = adc_avg[0];
//      uint16_t vcc = 6200 * 2;
    vcc = ((uint32_t)vin*vcc)>>13;
    uint8_t d[PACKET_LEN] = {vcc&0x7f, (vcc>>7)&0x7f, vin&0x7f, (vin>>7)&0x7f, 0};
    send(VOLTAGE, d);
}

void loop() {
//    TIMSK0 = 0;
    TIMSK1 = 0;
    TIMSK2 = 0;
    read_data();

    if(buzzer_timeout) {
        if(buzzer_timeout < millis()) {
            buzzer_timeout = 0;
            TIMSK2 = 0;
        }
    }

    read_analog();

    // send code up message on timeout
    static uint32_t t=0;
    uint32_t timeout[] = {0, 300, 350, 100};
    for(uint8_t source=1; source<4; source++) {
        uint32_t dt = t - codes[source].ltime;
        if(codes[source].lvalue && (dt > timeout[source] && dt < 10000)) {
            send_code(source, 0);
            codes[source].repeat_count = 0;
            codes[source].lvalue = 0;
            digitalWrite(LED_PIN, LOW);
        }
    }
    t = millis();
    // read from IR??
    /*
    if (ir.getResults()) {
        myDecoder.decode();
        send_code(IR, (myDecoder.value<<8) | myDecoder.protocolNum);
        ir.enableIRIn();      //Restart receiver
    }
    */

    if (rf.available()) {
        uint32_t value = rf.getReceivedValue();
        if(value && rf.getReceivedBitlength() == 24)
            send_code(RF, value);
        rf.resetAvailable();
    }

    // parse incoming data
    uint32_t dt = t - codes[GP].ltime;
    if(dt > 40) { // do not send faster than 40 ms
        for(int i=0; i<6; i++)
            if(!digitalRead(A0+i))
                send_code(GP, i+1);
        if(!digitalRead(7))
            send_code(GP, 7);
    }
}
