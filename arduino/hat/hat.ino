/* Copyright (C) 2020 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

/* this program reads rf on pin2, and ir on pin3 and outputs
   received remote presses as a spi slave */

#include <Arduino.h>
#include <stdint.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/boot.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#include <RCSwitch.h>

#define DATA_PIN 2
#define DIR_PIN 4
#define LED_PIN 8

#define USE_RF
#define USE_UART

// enable IR reception only for 328p (it has enough memory)
#ifdef __AVR_ATmega328P__
#define USE_IR
#endif

// start byte $ followed by PACKET_LEN bytes, and a parity byte
#define PACKET_LEN 6

#define VERSION_MAJOR  1
#define VERSION_MINOR  2

// of packet bytes, first byte defines message type 
enum {RF=0x01, IR=0x02, GP=0x03, VOLTAGE=0x04, ANALOG=0x05, VERSION=0x0a,
    SET_BACKLIGHT=0x16, SET_BUZZER=0x17, SET_BAUD=0x18, SET_ADC_CHANNELS=0x19, GET_VERSION=0x1b};

#ifdef USE_RF
RCSwitch rf = RCSwitch();
#endif

#ifdef USE_IR

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

#endif

uint8_t backlight_value = 64; // determines when backlight turns on
uint8_t backlight_polarity = 0;
uint8_t backlight_value_ee EEMEM = 64; // determines when backlight turns on
uint8_t backlight_polarity_ee EEMEM = 0;
uint8_t serial_baud_ee EEMEM = 0;

uint8_t adc_channels = 0;

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

RB_CREATE(serial_out, 124)
#ifdef USE_UART
RB_CREATE(serial_in, 124)

ISR(USART_RX_vect)
{
    UCSR0B &= ~_BV(RXCIE0);
    sei(); // needed because spi runs faster
    uint8_t x = UDR0;
    RB_PUT(serial_out, x);
    UCSR0B |= _BV(RXCIE0);
}

ISR(USART_UDRE_vect)
{
    UCSR0B &= ~_BV(UDRIE0);
//    sei();
    if(!RB_EMPTY(serial_in)) {
        RB_GET(serial_in, UDR0);
        UCSR0B |= _BV(UDRIE0);
    } else
        UCSR0B &= ~_BV(UDRIE0);
}
#endif

RB_CREATE(data_in, 124)

ISR (SPI_STC_vect) // SPI interrupt routine
{
    uint8_t c = SPDR;
    RB_GET(serial_out, SPDR);
    if(c>127)
        RB_PUT(data_in, c)
#ifdef USE_UART
    else if(c) {
        RB_PUT(serial_in, c);
        UCSR0B |= _BV(UDRIE0);
    }
#endif
}

/*
ISR(WDT_vect)
{
    wdt_reset();
    wdt_disable();

    asm volatile ("ijmp" ::"z" (0x0000)); // soft reset
}
*/
#ifdef USE_UART
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
#endif

uint32_t buzzer_timeout;
uint8_t buzzer_pitch, buzzer_pulse;
void buzzer_off()
{
    TIMSK2 = 0;
    pinMode(5, INPUT_PULLUP);
    pinMode(6, INPUT_PULLUP);
    PORTD &= ~_BV(PD5);
    PORTD &= ~_BV(PD6);
}

void set_buzzer(uint8_t mode, uint8_t timeout)
{
    if(buzzer_timeout)
        return;
    buzzer_timeout = millis() + timeout*10; // duration
    if(buzzer_timeout > 4294963200UL)
        buzzer_timeout = 1; // in case of 32 bit wrap.. needed???
        
    TCCR2A = _BV(WGM20) | _BV(WGM21);
    TCCR2B = _BV(WGM22) | _BV(CS22) | _BV(CS21); // divide by 256
    uint8_t freq;
    uint8_t pitch = mode&0x0f;
    buzzer_pulse = (mode&0xf0)>>4;
    switch(pitch) {
    case 1:        freq = 180;        break;
    case 2:        freq = 120;        break;
    case 3:        freq = 80;         break;
    case 4:        freq = 60;         break;
    case 5:        freq = 40;         break;
    case 6:        freq = 30;         break;
    case 7:        freq = 20;         break;
    case 8:        freq = 10;         break;
    default:
    case 0: // buzzer off
        buzzer_off();
        return;
    }
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);

    OCR2A = freq;
    OCR2B = freq>>1;
    
    TIMSK2 |= _BV(OCIE2B) | _BV(TOIE2);
}

void setup()
{
    // turn led on booting
    digitalWrite(LED_PIN, HIGH);
    pinMode(LED_PIN, OUTPUT);

    cli();
    wdt_reset();
    // 250 millisecond
    wdt_disable();
    WDTCSR = (1<<WDCE)|(1<<WDE);
    WDTCSR = 0;
    WDTCSR = (1<<WDCE)|(1<<WDE);
    WDTCSR = /*(1<<WDIE) |*/ (1<<WDE)|(1<<WDP2);
    sei();

    backlight_value = eeprom_read_byte(&backlight_value_ee);
    if(backlight_value == 0xff)
        backlight_value = 64;
    backlight_polarity = eeprom_read_byte(&backlight_polarity_ee);
    if(backlight_polarity == 0xff)
        backlight_polarity = 0;
    uint8_t baud = eeprom_read_byte(&serial_baud_ee);

#ifdef USE_UART
    Serial_begin(baud);
#endif
    pinMode(0, INPUT_PULLUP);
    pinMode(1, INPUT_PULLUP);

    // turn on SPI in slave mode
    SPCR |= _BV(SPE);
    // turn on interrupts
    SPCR |= _BV(SPIE);
    pinMode(MISO, OUTPUT);

    // buzzer
    pinMode(5, INPUT_PULLUP);
    pinMode(6, INPUT_PULLUP);
    pinMode(7, INPUT_PULLUP);

    pinMode(DATA_PIN, INPUT);
    pinMode(3, INPUT);
    pinMode(DIR_PIN, INPUT);

#ifdef USE_RF
    rf.enableReceive(0);  // Receiver on interrupt 0 => that is pin #2
#endif
#ifdef USE_IR
    ir.enableIRIn();
#endif

    for(int i=0; i<6; i++)
        pinMode(A0+i, INPUT_PULLUP);

    digitalWrite(9, LOW); /* enable internal pullups */
    pinMode(9, OUTPUT);

    if(MCUSR & 8) // beep on watchdog reset
        set_buzzer(2, 200);
    MCUSR = 0;

    digitalWrite(LED_PIN, LOW); // setup complete
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

struct codes_type {
    uint32_t lvalue, ltime;
    uint8_t repeat_count;
};

static struct codes_type codes[4] = {0};
static uint16_t adc_avg[6], adc_count, adc_cycles;
static uint32_t adc_a;

void send(uint8_t id, uint8_t d[PACKET_LEN])
{
    uint8_t st = '$'|0x80;
    cli();
    RB_PUT(serial_out, st);
    sei();
    id |= 0x80; // nmea does not use codes below 127, so this bit indicates data
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
        backlight_value = d[0];
        backlight_polarity = d[1];
        eeprom_write_byte(&backlight_value_ee, backlight_value);
        eeprom_write_byte(&backlight_polarity_ee, backlight_polarity);
        break;

    case SET_BUZZER:
        set_buzzer(data[1], data[2]);
        break;

    case SET_BAUD:
    {
        eeprom_update_byte(&serial_baud_ee, d[0]);
#ifdef USE_UART
        Serial_begin(d[0]);
#endif
    } break;
    case SET_ADC_CHANNELS:
    {
        adc_channels = d[0];
        if(adc_channels > 3)
            adc_channels = 0;
    } break;
    case GET_VERSION:
    {
        uint8_t d[PACKET_LEN] = {VERSION_MAJOR, VERSION_MINOR};
        send(VERSION, d);
    }
    }
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
#if 0        
        // unfortunately IR has infrequent but sometimes wrong value codes
        // so sending keyup from previous key if a new down is not used here.
        if(source == IR && codes[source].lvalue && value)
            return;
        // send code up for last key if key changed
        if(codes[source].lvalue) {
            uint8_t *plvalue = (uint8_t*)&codes[source].lvalue;
            uint8_t d[PACKET_LEN] = {plvalue[0], plvalue[1], plvalue[2], plvalue[3], 0, 0};
            send(source, d);
        }
#endif
        // send code up if not value
        if(codes[source].lvalue && !value) {
            uint8_t *plvalue = (uint8_t*)&codes[source].lvalue;
            uint8_t d[PACKET_LEN] = {plvalue[0], plvalue[1], plvalue[2], plvalue[3], 0, 0};
            send(source, d);
        }
        codes[source].repeat_count = 1;
    }
    
    if(cvalue) {
        uint8_t d[PACKET_LEN] = {pvalue[0], pvalue[1], pvalue[2], pvalue[3], codes[source].repeat_count, 0};
        send(source, d);
        digitalWrite(LED_PIN, HIGH); // turn on led to indicate remote received
    } else
        digitalWrite(LED_PIN, LOW);

    codes[source].lvalue = cvalue;
    codes[source].ltime = millis();
}

void read_analog()
{
    static uint8_t startup = 1;
    if(startup) {
        // 3 second delay to delay power draw of backlight
        if(millis() < 3000)
            return;
        // enable adc with 128 division
        ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
        ADCSRA |= _BV(ADSC);
        startup = 0;
    }
    
    static uint8_t channel;
    const uint8_t channels[] = {_BV(MUX1) | _BV(MUX2),              // 5v through divider
                                _BV(MUX0) | _BV(MUX1) | _BV(MUX2),  // ambient light sensor
                                _BV(MUX1) | _BV(MUX2) | _BV(MUX3),  // reference voltage
                                0,         // ADC0
                                _BV(MUX0), // ADC1
                                _BV(MUX1)  // ADC2
    };
    if(ADCSRA & _BV(ADSC))
        return; // not ready yet

    adc_a += ADCW;
    ADCSRA |= _BV(ADSC);
    if(++adc_count < 256)
        return;

    adc_avg[channel] = adc_a >> 4;
    adc_count = 0;
    adc_a = 0;

    if(++channel == (sizeof channels) - 3 + adc_channels)
        channel = 0;

    ADMUX = _BV(REFS0) | channels[channel]; // select channel at 5 volts
    ADCSRA |= _BV(ADSC);

    bool send_packet = false;
    for(int i=0; i<adc_channels; i++) {
        static uint16_t last_adc[3];
        if(abs(adc_avg[3+i] - last_adc[i]) > 25) {
            last_adc[i] = adc_avg[3+i];           
            send_packet = true;
            break;
        }
    }

    if(send_packet) {
        uint8_t d[PACKET_LEN] = {0};
        for(int i=0; i<adc_channels; i++) {
            d[2*i+0] = adc_avg[3+i]&0x7f;
            d[2*i+1] = (adc_avg[3+i]>>7)&0x7f;
        }
        send(ANALOG, d);
    }

    uint16_t ambient = adc_avg[1];
    //Configure TIMER1 to drive backlight variable pwm
    static uint8_t last_backlight = 0;
    uint8_t backlight = 0;
    if(ambient < 13000 ||
       (ambient < 15000 && last_backlight) ||
       backlight_value > 80)
        backlight = backlight_value;

    if(backlight != last_backlight) {
        last_backlight = backlight;
        if(backlight) {
            int ocr1a = backlight*backlight/11;
            //int ocr1a = 1000 - ambient/10;
            if(ocr1a < 0)
                ocr1a = 0;
            if(ocr1a > 1000)
                ocr1a = 1000;
            if(backlight_polarity)
                ocr1a = 1000 - ocr1a;

            OCR1A = ocr1a;
            TCCR1A=_BV(COM1A1)|_BV(WGM11);        //NON Inverted PWM
            TCCR1B=_BV(WGM13)|_BV(WGM12)|_BV(CS11); //PRESCALER=8 MODE 14(FAST PWM)
            ICR1 = 1000; // 1khz
        } else {
            TCCR1A=0;
            TCCR1B=0;
            TCNT1 = 0;
        }
    }

    if(++adc_cycles < 64)
        return;

    if(!RB_EMPTY(serial_out)) // don't report volts if other data is being sent
        return;

    uint32_t t = millis();
    for(uint8_t source=1; source<4; source++)
        if(t - codes[source].ltime < 500)
            return;
    
    adc_cycles = 0;
    uint16_t reference = adc_avg[2];
    
    // calculate input voltage (should be near 3.3)
    // reference/16/1023*Vin = 1.1
    // Vin = 1.1*1023*16*1000 / reference
    uint16_t vin = 18004800UL / reference; // in millivolts

    // calculate input power voltage through input divider
    // Vcc = 2*power/4/1023*vin
    uint16_t vcc = adc_avg[0];
    //      uint16_t vcc = 6200 * 2;
    vcc = ((uint32_t)vin*vcc)>>13;
    uint8_t d[PACKET_LEN];
    d[0] = vcc&0x7f;
    d[1] = (vcc>>7)&0x7f;
    d[2] = vin&0x7f;
    d[3] = (vin>>7)&0x7f;
    d[4] = 0;
    d[5] = 0;
    send(VOLTAGE, d);
}

void loop() {
    wdt_reset();
    read_data();
#if 0
    if(UCSR0A & _BV(UDRE0) && !RB_EMPTY(serial_in)) {
        cli();
        RB_GET(serial_in, UDR0);
        sei();
    }
#endif
#if 0
    if(UCSR0A & _BV(RXC0)) {
        cli();
        uint8_t x = UDR0;
        sei();
        RB_PUT(serial_out, x);
    }
#endif
    if(buzzer_timeout) {
        uint32_t t0 = millis();
        if(buzzer_timeout < t0) {
            buzzer_timeout = 0;
            buzzer_off();
        } else {
            if(buzzer_pulse == 1) {
                uint8_t pos = ((buzzer_timeout - t0) / 50)%16;
                if(pos < 7)
                    TIMSK2 = _BV(OCIE2B) | _BV(TOIE2);
                else
                    TIMSK2 = 0;
            } else
            if(buzzer_pulse == 2) {
                uint8_t pos = ((buzzer_timeout - t0) / 50)%16;
                if(pos & 1 && pos != 7 && pos != 15)
                    TIMSK2 = _BV(OCIE2B) | _BV(TOIE2);
                else
                    TIMSK2 = 0;
            } else
            if(buzzer_pulse == 3) {
                uint8_t pos = ((buzzer_timeout - t0) / 50)%16;
                if(pos & 1 && pos < 9)
                    TIMSK2 = _BV(OCIE2B) | _BV(TOIE2);
                else
                    TIMSK2 = 0;
            }
        }
    }
    read_analog();

    // send code up message on timeout
    static uint32_t t=0;
    t = millis();
#if 1
    uint32_t timeout[] = {0, 300, 350, 100};
    for(uint8_t source=1; source<4; source++) {
        uint32_t dt = t - codes[source].ltime;
        if(codes[source].lvalue && (dt > timeout[source] && dt < 10000)) {
            send_code(source, 0);
            if(source == RF)
                rf.resetAvailable();
        }
    }
#endif
#ifdef USE_IR
    // read from IR??
    if (ir.getResults()) {
        myDecoder.decode();
        if(myDecoder.protocolNum && myDecoder.bits >= 12)
            send_code(IR, (myDecoder.value<<8) | myDecoder.protocolNum);
        ir.enableIRIn();      //Restart receiver
    }
#endif
#ifdef USE_RF
    if (rf.available()) {
        uint32_t value = rf.getReceivedValue();
        if(value && rf.getReceivedBitlength() == 24) {
            if(value == 0x7c2933) // this specific code is used by pypilot remote for key up
                value = 0;            
            send_code(RF, value);
        }
        rf.resetAvailable();
    }
#endif
    
    // parse incoming data
    uint32_t dt = t - codes[GP].ltime;
    if(dt > 40) { // do not send faster than 40 ms
        uint8_t code = 0;
        for(int i=adc_channels; i<6; i++)
            if(!digitalRead(A0+i))
                code |= 1<<i;
        if(!digitalRead(7))
            code |= 1<<6;

        if(code)
            send_code(GP, code);
    }
}
