/*
 *   Adapted from original file from Eric Conner <Eric@EricConnerApps.com>
 */

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS            32
#define NUM_ANALOG_INPUTS           8
#define analogInputToDigitalPin(p)  ((p < 8) ? (p) + 24 : -1)
#define digitalPinHasPWM(p)         ((p) == 4 || (p) == 5 || (p) == 7 || (p) == 11)

#define PIN_SPI_SS    (12)
#define PIN_SPI_MOSI  (13)
#define PIN_SPI_MISO  (14)
#define PIN_SPI_SCK   (15)

static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

#define PIN_WIRE_SDA        (17)
#define PIN_WIRE_SCL        (16)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#define PIN_A0   (24)
#define PIN_A1   (25)
#define PIN_A2   (26)
#define PIN_A3   (27)
#define PIN_A4   (28)
#define PIN_A5   (29)
#define PIN_A6   (30)
#define PIN_A7   (31)

static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
static const uint8_t A2 = PIN_A2;
static const uint8_t A3 = PIN_A3;
static const uint8_t A4 = PIN_A4;
static const uint8_t A5 = PIN_A5;
static const uint8_t A6 = PIN_A6;
static const uint8_t A7 = PIN_A7;

#undef TCCR2A

#ifdef ARDUINO_MAIN

/*
                        ATMEL ATmega32

                          +---\\---+
       (XCK/T0) D8 PB0	01|        |40  PA0 AI0 D24 (ADC0)
           (T1) D9 PB1	02|        |39  PA1 AI1 D25 (ADC1)
   (INT2/AIN0) D10 PB2	03|        |38  PA2 AI2 D26 (ADC2)
    (OC0/AIN1) D11 PB3	04|        |37  PA3 AI3 D27 (ADC3)
          (SS) D12 PB4	05|        |36  PA4 AI4 D28 (ADC4)
        (MOSI) D13 PB5	06|        |35  PA5 AI5 D29 (ADC5)
        (MISO) D14 PB6	07|        |34  PA6 AI6 D30 (ADC6)
         (SCK) D15 PB7	08|        |33  PA7 AI7 D31 (ADC7)
                 RESET	09|        |32  AREF
                   VCC	10|        |31  GND
                   GND	11|        |30  AVCC
                 XTAL2	12|        |29  PC7 D23 (TOSC2)
                 XTAL1	13|        |28  PC6 D22 (TOSC1)
          (RXD) D0 PD0	14|        |27  PC5 D21 (TDI)
          (TXD) D1 PD1	15|        |26  PC4 D20 (TDO)
         (INT0) D2 PD2	16|        |25  PC3 D19 (TMS)
         (INT1) D3 PD3	17|        |24  PC2 D18 (TCK)
         (OC1B) D4 PD4	18|        |23  PC1 D17 (SDA)
         (OC1A) D5 PD5	19|        |22  PC0 D16 (SCL)
         (ICP1) D6 PD6	20|        |21  PD7 D7 (OC2)
                          +--------+
*/

const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &DDRA,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
	NOT_A_PORT,
	NOT_A_PORT,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PORTA,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
	NOT_A_PORT,
	NOT_A_PORT,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PINA,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
	NOT_A_PORT,
	NOT_A_PORT,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[32] = {
	PD,  // PD0 ** D0
	PD,  // PD1 ** D1
	PD,  // PD2 ** D2
	PD,  // PD3 ** D3
	PD,  // PD4 ** D4
	PD,  // PD5 ** D5
	PD,  // PD6 ** D6
	PD,  // PD7 ** D7
	PB,  // PB0 ** D8
	PB,  // PB1 ** D9
	PB,  // PB2 ** D10
	PB,  // PB3 ** D11
	PB,  // PB4 ** D12
	PB,  // PB5 ** D13
	PB,  // PB6 ** D14
	PB,  // PB7 ** D15
	PC,  // PC0 ** D16
	PC,  // PC1 ** D17
	PC,  // PC2 ** D18
	PC,  // PC3 ** D19
	PC,  // PC4 ** D20
	PC,  // PC5 ** D21
	PC,  // PC6 ** D22
	PC,  // PC7 ** D23
	PA,  // PA0 ** A0 D24
	PA,  // PA1 ** A1 D25
	PA,  // PA2 ** A2 D26
	PA,  // PA3 ** A3 D27
	PA,  // PA4 ** A4 D28
	PA,  // PA5 ** A5 D29
	PA,  // PA6 ** A6 D30
	PA,  // PA7 ** A7 D31
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[32] = {
	_BV(0),  // PD0 ** D0
	_BV(1),  // PD1 ** D1
	_BV(2),  // PD2 ** D2
	_BV(3),  // PD3 ** D3
	_BV(4),  // PD4 ** D4
	_BV(5),  // PD5 ** D5
	_BV(6),  // PD6 ** D6
	_BV(7),  // PD7 ** D7
	_BV(0),  // PB0 ** D8
	_BV(1),  // PB1 ** D9
	_BV(2),  // PB2 ** D10
	_BV(3),  // PB3 ** D11
	_BV(4),  // PB4 ** D12
	_BV(5),  // PB5 ** D13
	_BV(6),  // PB6 ** D14
	_BV(7),  // PB7 ** D15
	_BV(0),  // PC0 ** D16
	_BV(1),  // PC1 ** D17
	_BV(2),  // PC2 ** D18
	_BV(3),  // PC3 ** D19
	_BV(4),  // PC4 ** D20
	_BV(5),  // PC5 ** D21
	_BV(6),  // PC6 ** D22
	_BV(7),  // PC7 ** D23
	_BV(7),  // PA0 ** A0 D24
	_BV(6),  // PA1 ** A1 D25
	_BV(5),  // PA2 ** A2 D26
	_BV(4),  // PA3 ** A3 D27
	_BV(3),  // PA4 ** A4 D28
	_BV(2),  // PA5 ** A5 D29
	_BV(1),  // PA6 ** A6 D30
	_BV(0),  // PA7 ** A7 D31
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[32] = {
	NOT_ON_TIMER,  // PD0 ** D0
	NOT_ON_TIMER,  // PD1 ** D1
	NOT_ON_TIMER,  // PD2 ** D2
	NOT_ON_TIMER,  // PD3 ** D3
	TIMER1B,       // PD4 ** D4
	TIMER1A,       // PD5 ** D5
	NOT_ON_TIMER,  // PD6 ** D6
	TIMER2,	       // PD7 ** D7
	NOT_ON_TIMER,  // PB0 ** D8
	NOT_ON_TIMER,  // PB1 ** D9
	NOT_ON_TIMER,  // PB2 ** D10
	TIMER0A,       // PB3 ** D11
	NOT_ON_TIMER,  // PB4 ** D12
	NOT_ON_TIMER,  // PB5 ** D13
	NOT_ON_TIMER,  // PB6 ** D14
	NOT_ON_TIMER,  // PB7 ** D15
	NOT_ON_TIMER,  // PC0 ** D16
	NOT_ON_TIMER,  // PC1 ** D17
	NOT_ON_TIMER,  // PC2 ** D18
	NOT_ON_TIMER,  // PC3 ** D19
	NOT_ON_TIMER,  // PC4 ** D20
	NOT_ON_TIMER,  // PC5 ** D21
	NOT_ON_TIMER,  // PC6 ** D22
	NOT_ON_TIMER,  // PC7 ** D23
	NOT_ON_TIMER,  // PA0 ** A0 D24
	NOT_ON_TIMER,  // PA1 ** A1 D25
	NOT_ON_TIMER,  // PA2 ** A2 D26
	NOT_ON_TIMER,  // PA3 ** A3 D27
	NOT_ON_TIMER,  // PA4 ** A4 D28
	NOT_ON_TIMER,  // PA5 ** A5 D29
	NOT_ON_TIMER,  // PA6 ** A6 D30
	NOT_ON_TIMER,  // PA7 ** A7 D31
};

#endif
#endif
