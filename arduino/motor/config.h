/*
 * Author: Timo Birnschein (timo.birnschein@googlemail.com)
 * Date: 2019/11/24
 * Credits: Where they are due: https://github.com/seandepagnier is the original code writer
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

/*You may need to modify the source code to support different hardware

adc pin0 is a resistor divider to measure voltage
             allowing up to 20 volts (10k and 560 ohm, 1.1 volt reference)
adc pin1 goes to .01/.05 ohm shunt to measure current
adc pin2 goes to 100k resistor to 5v and 10k NTC thermistor to gnd ctrl temp
adc pin3 goes to 100k resistor to 5v and 10k NTC thermistor to gnd motor temp
adc pin4 rudder sense
unused analog pins should be grounded

digital pins 4 and 5 determine the current sense as folows:
pin 4 determines range
pin 5 determines high/low current  (20A or 60A max)

D4  D5
 1   1        .05 ohm, (or .001 ohm x 50 gain)
 0   1        .01 ohm
 1   0        .0005 ohm x 50 gain
 0   0        .0005 ohm x 200 gain   *ratiometric mode


digital pin6 determines:
1 - RC pwm:
   digital pin9 pwm output standard ESC (1-2 ms pulse every 20 ms)
           pin2 esc programming input/output (with arduinousblinker script)
0 - Hbridge
   digital pin2 and pin3 for low side, pin9 and pin10 for high side


optional:digital pin7 forward fault for optional switch to stop forward travel
digital pin8 reverse fault for optional switch to stop reverse travel


Ratiometric Mode:
for D4=0 and D5=0, the adc operates over the 0-5 volt range
making it ratiometric (linearly accurate) for rudder feedback
and reduces impedance in the rudder measurement
the temperature resistors are changed to 10k and 10k ntc
voltage measurement accuracy is reduced, and the resistors used are
15k and 100k for a range of 38 volts.   Pin 12 is not used in this mode.


Pin 11 drives mosfet (560ohm and 10k resistors) for clutch engage.

If Pin 12 has 560 ohm resistor to A0, then 24 volts is supported,
this allows for measuring voltage up to 40.4 volts

D12
 1    0-20.75 volts (560 and 10k resistor)  resolution 0.02 volts
 0    0-40.4  volts (280 and 10k resistor)  resolution 0.04 volts

digital pin13 is led on when engaged


The program uses a simple protocol to ensure only
correct data can be received and to ensure that
false/incorrect or random data is very unlikely to
produce motor movement.

The input and output over uart has 4 byte packets

The first byte is the command or register, the next
two bytes is a 16bit value (signed or unsigned)
the last byte is a crc8 of the first 3 bytes

If incoming data has the correct crc for a few frames
the command can be recognized.

*/

/* vnh2sp30 is supported, but warning, I received 3 boards:
1) reverse is half power making chip very hot
2) reverse does not work
3) current sense does not work

3 out of 3 were defective, I do not recommend.

vnh2sp30  <->  arduino <->  CPC5001
+5V              5v
GND              GND
EN               D10
CS               A1
INA              D2
INB              D3
PWM              D9

If used with optical isolation (strongly recommended)
PWR+             VIN
                 5v         vcc
                 tx         rx
                 rx         tx
                 gnd        gnd
*/

enum commands {COMMAND_CODE=0xc7, RESET_CODE=0xe7, MAX_CURRENT_CODE=0x1e, MAX_CONTROLLER_TEMP_CODE=0xa4, MAX_MOTOR_TEMP_CODE=0x5a, RUDDER_RANGE_CODE=0xb6, RUDDER_MIN_CODE=0x2b, RUDDER_MAX_CODE=0x4d, REPROGRAM_CODE=0x19, DISENGAGE_CODE=0x68, MAX_SLEW_CODE=0x71, EEPROM_READ_CODE=0x91, EEPROM_WRITE_CODE=0x53};

enum results {CURRENT_CODE=0x1c, VOLTAGE_CODE=0xb3, CONTROLLER_TEMP_CODE=0xf9, MOTOR_TEMP_CODE=0x48, RUDDER_SENSE_CODE=0xa7, FLAGS_CODE=0x8f, EEPROM_VALUE_CODE=0x9a};

enum {SYNC=1, OVERTEMP_FAULT=2, OVERCURRENT_FAULT=4, ENGAGED=8, INVALID=16*1, PORT_PIN_FAULT=16*2, STARBOARD_PIN_FAULT=16*4, BADVOLTAGE_FAULT=16*8, MIN_RUDDER_FAULT=256*1, MAX_RUDDER_FAULT=256*2, CURRENT_RANGE=256*4, BAD_FUSES=256*8};


enum {CONTROLLER_TEMP, MOTOR_TEMP};

/*
 * BOARD TYPE
 */
//#define BOARD_VNH2SP30 // defined if this board is used
#define BOARD_IBT2_H_BRIDGE // defined if this board is used IBT-2 H-Bridge
//#define BOARD_RAW_H_BRIDGE // defined if a raw h-bridge is used that needs to be controlled directly

/*
 * ATTACHED SENSORS
 */
//#define DISABLE_TEMP_SENSE    // if no temp sensors avoid errors
//#define DISABLE_VOLTAGE_SENSE // if no voltage sense
//#define DISABLE_CURRENT_SENSE // if no motor current sensor is installed or used
//#define DISABLE_RUDDER_SENSE  // if no rudder sense
#define DISABLE_ENDSTOPS // if no endstops are installed we won't have a forward and reverse faults

//#define DISABLE_DEBUGGING_DISPLAY // If a debugging TFT display is attached to the controller, comment this out
#define USE_STEINHART_TEMP_SENSING // If you want to use the much slower Steinhart calculation

/*
 * Current configuration
 */
#define LOW_CURRENT // gives 2000 mA. Comment to get 4000 mA max current


// *************************************************************************************************** //
// ********************************* GENERAL CONFIGURATION VALUES ************************************ //
// *************************************************************************************************** //
#define PWM_DEADBAND 40

#define RUDDER_MIN_ADC 0
#define RUDDER_MAX_ADC 65535
#define TEMPERATURE_CONTROLLER_MAX 7000
#define TEMPERATURE_MOTOR_MAX 7000
#define CURRENT_MOTOR_MAX 2000
#define SPEEDUP_SLEW_RATE 15
#define SLOWDOWN_SLEW_RATE 30


// *************************************************************************************************** //
// ********************************* Thermocouple Variables ****************************************** //
// *************************************************************************************************** //
// Check https://learn.adafruit.com/thermistor/using-a-thermistor for more info
// resistance at 25 degrees C
#define THERMISTORNOMINAL 100000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 4700

// *************************************************************************************************** //
// ********************************* Battery Voltage Measurement Variables *************************** //
// *************************************************************************************************** //
// Maximum input voltage on the supply pin
#define VIN_MAX 1800
// Maximum input voltage on the supply pin
#define VIN_MIN 900
// Wow the ADC is configured to measure: 0V - 5V
#define ADC_RANGE 5.0f
// Bit deapth of the ADC being used
#define BIT_DEPTH 1024.0f
// Calculated voltage separation per bit
#define V_SEPARATION ADC_RANGE / BIT_DEPTH
// Voltage divider resistor 1
#define R1_1 22000.0f
// Voltage divider resistor 2
#define R2_1 4700.0f
// Constant calculated from resistor values for simplified voltage calculation
#define RESISTOR_CONSTANT_1 (R2_1/(R1_1 + R2_1))

#endif
