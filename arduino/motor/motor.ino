/* Copyright (C) 2019 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

 /*
  * Modified by Timo Birnschein (timo.birnschein@googlemail.com)
  * Start Date: 11/23/2019
  * Current State: Pretty hard to read code. Insufficient documentation. Difficult to maintain.
  * Desired changes: Must work with IBT-2 H-Bridge as these are widely available and easy to use.
  * 
  * Notes:  I don't believe anyone with access to the code (as there is no documentation available)
  *         would use external jumpers to configure this. A config.h would be preferable.
  *         I will eliminate all this external configuration stuff and concentrate on making this
  *         more human readible.
  *         I will concentrate on the position, engage and disengage functions to implement the IBT-2 driver.
  */

#include <Arduino.h>
#include <stdint.h>
#include <stdarg.h>
#include <HardwareSerial.h>

#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/boot.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "pins.h"
#include "config.h"
#include "crc.h"
#include "adc_filtering.h"
#include "status_display.h"
/*
 * This program is meant to interface with pwm based motor controller either brushless or brushed, or a regular RC servo
 */

/*
 ******************************** Global Variables *************************************
 */

uint16_t flags = 0;
uint8_t serialin, packet_count = 0;

// These variables hold values that can be changed by the controlling PyPilot remotely.
uint16_t max_current = CURRENT_MOTOR_MAX; // 20 Amps
uint16_t max_controller_temp= TEMPERATURE_CONTROLLER_MAX; // 70C
uint16_t max_motor_temp = TEMPERATURE_MOTOR_MAX; // 70C
uint8_t max_slew_speed = SPEEDUP_SLEW_RATE;
uint8_t max_slew_slow = SLOWDOWN_SLEW_RATE; // 200 is full power in 1/10th of a second
uint16_t rudder_min = RUDDER_MIN_ADC;
uint16_t rudder_max = RUDDER_MAX_ADC; // Analog rudder value between -100 and 100 full scale.

uint8_t eeprom_read_addr = 0;
uint8_t eeprom_read_end = 0;

uint8_t in_bytes[3];
uint8_t sync_b = 0, in_sync_count = 0;

uint8_t out_sync_b = 0, out_sync_pos = 0;
uint8_t crcbytes[3];

uint8_t timeout = 0;
uint32_t last_loop_cycle_millis = 0;
uint32_t last_loop_rudder_millis = 0;
uint32_t last_loop_voltage_millis = 0;
uint32_t last_loop_current_millis = 0;
uint32_t last_loop_temperature_millis = 0;

/*
 * command_value is used throughout the program to hold the rudder motor PWM between 0 and 2000 with 0 being PWM off.
 */
uint16_t command_value = 1000;

/* 
 * command is from 0 to 2000 with 1000 being neutral 
 */
uint16_t lastpos = 1000;

void setup() // Must change completely
{

#ifndef DISABLE_DEBUGGING_DISPLAY
    display_init();
#endif

    // Disable all interrupts
    cli();

/* 
 *  Clear MCU Status Register. Not really needed here as we don't need to know why the MCU got reset. 
 *  page 44 of datasheet
*/
    MCUSR = 0;

/* 
 *  Disable and clear all Watchdog settings. Nice to get a clean slate when dealing with interrupts 
*/
    WDTCSR = (1<<WDCE)|(1<<WDE);

    /*
     *  Watchdog interrupt fires every 1/4th second
     */
    WDTCSR = (1<<WDIE) | (1<<WDP2);

    /*
     * read fuses, and report this as flag if they are wrong
     */
    uint8_t lowBits      = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
    uint8_t highBits     = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
    uint8_t extendedBits = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
    // uint8_t lockBits     = boot_lock_fuse_bits_get(GET_LOCK_BITS); // too many clones don't set lock bits and there is no spm

    if(lowBits != 0xFF || highBits != 0xda ||
       (extendedBits != 0xFD && extendedBits != 0xFC)
       // || lockBits != 0xCF // too many clones don't set lock bits and there is no spm
        )
        flags |= BAD_FUSES;

    sei(); // Enable all interrupts.

    // Todo: Not sure what serialin is used for. Looks like packet counter, buut that exists as well
    serialin = 0;
    // set up Serial library
    Serial.begin(38400);

    set_sleep_mode(SLEEP_MODE_IDLE); // wait for serial

#ifndef DISABLE_ENDSTOPS
    pinMode(STBD_FAULT_PIN, INPUT);
    pinMode(PORT_FAULT_PIN, INPUT);
    digitalWrite(STBD_FAULT_PIN, HIGH); /* enable internal pullups */
    digitalWrite(PORT_FAULT_PIN, HIGH); /* enable internal pullups */
#endif

    pinMode(ENGAGE_LED_PIN, OUTPUT); // status LED
    digitalWrite(ENGAGE_LED_PIN, LOW);

    _delay_us(100); // time to settle

#ifdef BOARD_IBT2_H_BRIDGE
    digitalWrite(RPWM_PIN, LOW); /* disable internal pullups */
    digitalWrite(LPWM_PIN, LOW); /* disable internal pullups */
    digitalWrite(ENABLE_PIN, LOW); /* disable internal pullups */
    digitalWrite(R_IS_PIN, LOW); /* disable internal pullups */
    digitalWrite(L_IS_PIN, LOW); /* disable internal pullups */
    
    // Configure as output and drive the pin low
    pinMode(RPWM_PIN, OUTPUT); // PWM for half bridge driver
    pinMode(LPWM_PIN, OUTPUT); // PWM for half bridge driver
    pinMode(ENABLE_PIN, OUTPUT); // Enables or disables both half bridges
    pinMode(R_IS_PIN, INPUT); // Analog current sense and fault detection if used
    pinMode(L_IS_PIN, INPUT); // Analog current sense and fault detection if used
#endif

    // setup adc
    DIDR0 = 0x3f; // disable all digital io on analog pins
}

/*
 * Configure all PWM modes for the different hardware and set a position command
 */
void engage() // Must change completely
{
    if(flags & ENGAGED) { // Already engaged
        //update_command(); // 30hz
        return;
    }

#ifdef BOARD_IBT2_H_BRIDGE
    pinMode(RPWM_PIN, OUTPUT); // Ensure the pins are configured correctly
    pinMode(LPWM_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    
    analogWrite(RPWM_PIN, 0); // Disengage PWM to both h-bridges
    analogWrite(LPWM_PIN, 0); // Disengage PWM to both h-bridges
    digitalWrite(ENABLE_PIN, HIGH); // Enable both half bridges
#endif

    position(1000); // Not sure why position 1000 is called.
    digitalWrite(ENGAGE_LED_PIN, HIGH); // status LED
    
    timeout = 0;
    flags |= ENGAGED;

#ifndef DISABLE_DEBUGGING_DISPLAY
  display_motor_engaged();
#endif
}

/*
 * Take the desired value, compare with current value and create a new desired value
 * based on the allowed slew rate to move the rudder not faster than deemed safe.
 */
 
#ifndef DISABLE_DEBUGGING_DISPLAY

#endif
void update_command() // Will not be changed
{
    int16_t speed_rate  = max_slew_speed;
    int16_t slow_rate   = max_slew_slow; // value of 20 is 1 second full range at 50hz
    uint16_t cur_value  =  lastpos;
    int16_t diff        = (int)command_value - (int)cur_value;

    // limit motor speed change to stay within speed and slow slew rates
    if(diff > 0) {
        if(cur_value < 1000) {
            if(diff > slow_rate)
               diff = slow_rate;
        } else
            if(diff > speed_rate)
                diff = speed_rate;
    } else {
        if(cur_value > 1000) {
            if(diff < -slow_rate)
                diff = -slow_rate;
        } else
            if(diff < -speed_rate)
                diff = -speed_rate;
    }

    // Push the new value over to the position function
    position(cur_value + diff);
}

/*
 * This is where the magic happens. Receive a command value and drive the corresponding motor controller
 * using the desired pwm setting.
 */

void position(uint16_t value)
{
  // store the new value as the new last position, used by update_command()
  lastpos = value;
  int16_t newValue = abs((int)value - 1000);
  
  // Determine if value is bigger or smaller than 1000 plus deadzone
  if(value > 1000 + PWM_DEADBAND) {
    // Turn PWM for CCW operation on and the other off
    //analogWrite(RPWM_PIN, abs((int)value - 1000));
    analogWrite(RPWM_PIN, (uint8_t)((255.0f/1000.0f) * (float)newValue));
    analogWrite(LPWM_PIN, 0);
  } else if(value < 1000 - PWM_DEADBAND) {
    // Turn PWM for CW operation on and the other off
    analogWrite(RPWM_PIN, 0);
    //analogWrite(LPWM_PIN, abs((int)value - 1000));
    analogWrite(LPWM_PIN, (uint8_t)((255.0f/1000.0f) * (float)newValue));
  } else {
      // Nothing to do. We got about 1000 and need to turn the breaks on and PWM off
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, 0);
  }
  
#ifndef DISABLE_DEBUGGING_DISPLAY
  display_motor_PWM = newValue; 
#endif
}

/*
 * Reset the rudder position back to center
 */
void stop()
{
    position(1000);
    command_value = 1000;
}

/*
 * Just stop. That's literally what these two functions do. There is no difference unless it's exactly 1000.
 * Then it does nothing.
 */
void stop_port()
{
    if(lastpos > 1000)
       stop();
}

void stop_starboard()
{
    if(lastpos < 1000)
       stop();
}

/*
 * Stops the servo, sets the desired value to 1000 (center), opens the clutch and kills the LED
 */
void disengage() // Will not be changed
{
    stop();
    flags &= ~ENGAGED;
    timeout = 60; // detach in about 62ms
    digitalWrite(ENGAGE_LED_PIN, LOW); // status LED

#ifndef DISABLE_DEBUGGING_DISPLAY
  display_motor_disengaged();
#endif
}

/*
 * Todo: Purpose unclear. It seems to deactivate the h-bridge in all different pwm_styles.
 * However, for the RC_Servo and an h-bridge this should be different. 
 * Todo: Find out if this function is only used when an H-Bridge is being utilized.
 * Todo: Make this more obvious. The naming does not give this away.
 */
void detach() // Must change completely
{
  #ifdef BOARD_IBT2_H_BRIDGE
    analogWrite(RPWM_PIN, 0); // Disengage PWM to both h-bridges
    analogWrite(LPWM_PIN, 0); // Disengage PWM to both h-bridges
    digitalWrite(ENABLE_PIN, LOW); // Disable both H-Bridges. This will also disable the motor brake
  #endif
  
  timeout = 80; // avoid overflow
}

/*
 * UNTESTED!
 * Takes a filtered measurement and converts it into the format PPyPilot expects to send it out.
 */
uint16_t TakeAmps()
{
    uint32_t amps = getADCFilteredValue(MOTOR_CURRENT);
    amps = amps * 9 / 34 / 16;
    
#ifndef DISABLE_DEBUGGING_DISPLAY
    display_motor_current = amps;
#endif

    return amps;
}


/*
 * This function returns a filtered supply voltage measurement
 */
uint16_t TakeVolts()
{
  uint16_t voltage;
    // Calculate a float value and then bring it into the format expected by PyPilot: 18V = 1800
    voltage = (uint16_t)((getADCFilteredValue(SUPPLY_VOLTAGE) * V_SEPARATION) / RESISTOR_CONSTANT_1 * 100.0f);

#ifndef DISABLE_DEBUGGING_DISPLAY
    display_supply_voltage = voltage;
#endif
    
    return voltage;
}

/*
 * This function returns either controller or motor temperature.
 * Equations based on https://learn.adafruit.com/thermistor/using-a-thermistor
 * However, the original fixed point math version can also be used. See config.h for details
 */
uint16_t TakeTemp(uint8_t index)
{
#ifdef USE_STEINHART_TEMP_SENSING
  float adc_temp_value = 0;
  float steinhart = 0;

  uint16_t temperature = 0;
    
  switch (index)
  {
    case CONTROLLER_TEMP:
      adc_temp_value = getADCFilteredValue(CONTROLLER_TEMPERATURE);
      break;
    case MOTOR_TEMP:
      adc_temp_value = getADCFilteredValue(MOTOR_TEMPERATURE);
      break;
    default:
      adc_temp_value = 0; // Some out of bounds value was given.
      break;
  }
  
    // convert the value to resistance
    adc_temp_value = 1023.0f / adc_temp_value - 1;
    adc_temp_value = SERIESRESISTOR / adc_temp_value;
    
    steinhart = adc_temp_value / THERMISTORNOMINAL;       // (R/Ro)
    steinhart = log(steinhart);                           // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                            // 1/B * ln(R/Ro)
    steinhart += 1.0f / (TEMPERATURENOMINAL + 273.15);     // + (1/To)
    steinhart = 1.0f / steinhart;                          // Invert
    steinhart -= 273.15f;                                  // convert to C
    
    temperature = (uint16_t)(steinhart * 100.0f);

#ifndef DISABLE_DEBUGGING_DISPLAY
  switch (index)
  {
    case CONTROLLER_TEMP:
      display_controller_temperature = temperature;
      break;
    case MOTOR_TEMP:
      display_motor_temperature = temperature;
      break;
    default:
      break;
  }
#endif

    return temperature;
#else
  uint16_t adc_temp_value = 0, R;
  
  switch (index)
  {
    case CONTROLLER_TEMP:
      adc_temp_value = getADCFilteredValue(CONTROLLER_TEMPERATURE);
      R = 10000 * adc_temp_value / (16384 - adc_temp_value);  // resistance in ohms
      temperature = 30000000 / (R + 2600) + 200;
#ifndef DISABLE_DEBUGGING_DISPLAY
      display_controller_temperature = temperature;
#endif
      return temperature;
    case MOTOR_TEMP:
      adc_temp_value = getADCFilteredValue(MOTOR_TEMPERATURE);
      R = 10000 * adc_temp_value / (16384 - adc_temp_value);  // resistance in ohms
      temperature = 30000000 / (R + 2600) + 200;
#ifndef DISABLE_DEBUGGING_DISPLAY
      display_motor_temperature = temperature;
#endif
      return temperature;
    default:
      adc_temp_value = 0; // Some out of bounds value was given.
      break;
  }
#endif
}

/*
 * Returns internal temperature of MCU
 * NOT IMPLEMENTED - DEPRICATED. Needs reimplementation
 */
uint16_t TakeInternalTemp()
{

    return 0;
}

/*
 * This function, returns a filtered rudder angle between 0 and 65535 to PyPilot.
 * Todo:  Needs scaling and offset! Currently, whatever value was measured at the potentiometer will be sent to PyPilot.
 *        This results in values from -15 to 22 degrees which is not at all desireable.
 */
uint16_t TakeRudder()
{
  //uint16_t value = getADCFilteredValue(RUDDER_ANGLE);
  uint16_t value = rescaleValue16(getADCFilteredValue(RUDDER_ANGLE), RUDDER_MIN_ADC, RUDDER_MAX_ADC, 65535);
#ifndef DISABLE_DEBUGGING_DISPLAY
      display_sensor_ADC = value;
#endif
    return value;
}

/* 
 * This is the interrupt that is executed when the watch dog timer fires.
 * It also restarts the program by jumping back to start of code.
 * Todo: Will anyone be informed if this happens? There is no start code feedback over serial.
 * In other words, it just restarts and no one will ever know this happened in the first place.
 */
ISR(WDT_vect)
{
    wdt_reset();
    wdt_disable();
    disengage();
    delay(50);
    detach();

    asm volatile ("ijmp" ::"z" (0x0000));
}

/*
 * Read incoming packets, understand what needs to be done and execute command.
 * Byte 0 contains the command type.
 * Byte 1 and 2 contain the 16 bit value and needed to be OR'ed into one word.
 */ 

void process_packet()
{
    flags |= SYNC;
    uint16_t value = in_bytes[1] | in_bytes[2]<<8;

    switch(in_bytes[0]) {
    case REPROGRAM_CODE:
    {
        // jump to bootloader
        asm volatile ("ijmp" ::"z" (0x3c00));
        //goto *0x3c00;
    } break;
    case RESET_CODE:
        // reset overcurrent flag
        flags &= ~OVERCURRENT_FAULT;
        break;
    case COMMAND_CODE:
        timeout = 0; // Reset timeout to make sure we're not resetting anything else
        if(serialin < 12)
            serialin+=4; // output at input rate
        if(value > 2000); // out of range (can only be positive because it's a uint16_t)
            // unused range, invalid!!!
            // ignored
        else if(flags & (OVERTEMP_FAULT | OVERCURRENT_FAULT | BADVOLTAGE_FAULT));
            // no command because of overtemp or overcurrent or badvoltage
        else if((flags & (PORT_PIN_FAULT | MAX_RUDDER_FAULT)) && value > 1000)
            stop();
            // no forward command if port fault
        else if((flags & (STARBOARD_PIN_FAULT | MIN_RUDDER_FAULT)) && value < 1000)
            stop();
            // no starboard command if port fault
        else {
            command_value = value;
#ifndef DISABLE_DEBUGGING_DISPLAY
            display_motor_command = value;
#endif
            engage();
        }
        break;
    case MAX_CURRENT_CODE: { // current in units of 10mA
      /*
       * Todo: Needs reimplpementation. Since I removed all external configuration this is depricated for this version of the software
       */
#ifdef LOW_CURRENT
        unsigned int max_max_current = 2000;
#else
        unsigned int max_max_current = 4000;
#endif
        if(value > max_max_current) // maximum is 20 or 40 amps
            value = max_max_current;
        max_current = value;
    } break;
    case MAX_CONTROLLER_TEMP_CODE:
        if(value > 10000) // maximum is 100C
            value = 10000;
        max_controller_temp = value;
        break;
    case MAX_MOTOR_TEMP_CODE:
        if(value > 10000) // maximum is 100C
            value = 10000;
        max_motor_temp = value;
        break;
#if 0 // ignore obsolete rudder range code // Timo: Yes, shifting makes no sense here
    case RUDDER_RANGE_CODE:
      rudder_max = in_bytes[1]<<8;
      rudder_min = in_bytes[2]<<8;
      break;
#endif
    case RUDDER_MIN_CODE:
        rudder_min = value; // Todo: I need to understand what is happening with this value. It seems like it's not being used. Especially because it's not being stored or loaded at boot.
        break;
    case RUDDER_MAX_CODE:
        rudder_max = value; // Todo: I need to understand what is happening with this value. It seems like it's not being used. Especially because it's not being stored or loaded at boot.
        break;
    case DISENGAGE_CODE:
        if(serialin < 12)
            serialin+=4; // output at input rate
        disengage();
        break;
    case MAX_SLEW_CODE: {
        max_slew_speed = in_bytes[1];
        max_slew_slow = in_bytes[2];

        // if set at the end of range (up to 255)  no slew limit
        if(max_slew_speed > 250)
            max_slew_speed = 250;
        if(max_slew_slow > 250)
            max_slew_slow = 250;

        // must have some slew
        if(max_slew_speed < 1)
            max_slew_speed = 1;
        if(max_slew_slow < 1)
            max_slew_slow = 1;
    } break;

    case EEPROM_READ_CODE:
        if(eeprom_read_addr == eeprom_read_end) {
            eeprom_read_addr = in_bytes[1];
            eeprom_read_end = in_bytes[2];
        }        
    break;

    case EEPROM_WRITE_CODE:
        eeprom_update_8(in_bytes[1], in_bytes[2]);
    break;
    }
}

/*
 * Main loop. Now it gets interesting.
 */
void loop() // Must change
{    
    static uint32_t last_loop_cycle_millis = 0;
    static uint32_t adc_filter_cycle_millis = 0;
    wdt_reset(); // strobe watchdog

/*
 * We need to update the command to drive the rudder to the desired position at the desired rate,
 * but only if the controller is ENGAGED
 */
    if(millis() - last_loop_cycle_millis > 20) {
        if(flags & ENGAGED) {
            static uint8_t update_d;
            if(++update_d >= 4) {
                update_command(); // run the update function at roughly 30hz
                update_d = 0;
            }
        }

        timeout++;
        last_loop_cycle_millis = millis(); // Store the time from here to next iteration
    }

    if(millis() - adc_filter_cycle_millis > 10)
    {
      ADC_updateAndFilter(); // place another ADC value for each configured channel into the corresponding ring buffer
      adc_filter_cycle_millis = millis(); // Store the time from here to next iteration
    }

/*
 * If we're not ENGAGED (DISENGAGED), the system will slowly shut down
 */
    if(timeout == 120)
        disengage();

/*
 * And finally disengage the H-Bridge completely
 */
    if(timeout >= 128) // detach 160 ms later so esc gets stop
        detach();

#ifndef DISABLE_DEBUGGING_DISPLAY

  display_flags = flags;
/*
 * Display update for status variables
 */
  display_update();

#endif

    /*
     * **************************** SERIAL INPUT ***********************************************
     * 1. Read 3 bytes
     * 2. Check if the fourth byte is a valid CRC of the first three.
     * 3. If yes, check if at least three packets in a row were valid, if so process packet.
     * 4. if no, discard, disengage, set invalid flag, reset everything.
     */
    while(Serial.available()) {
      uint8_t c = Serial.read();
      if(sync_b < 3) {
          in_bytes[sync_b] = c;
          sync_b++;
      } else {
          if(c == crc8(in_bytes, 3)) {
              if(in_sync_count >= 2) { // if crc matches, we have a valid packet
                  process_packet();
              } else
                  in_sync_count++;

              sync_b = 0;
              flags &= ~INVALID;
          } else {
              // invalid packet
              flags &= ~SYNC;
              stop(); //disengage();
              in_sync_count = 0;
              packet_count = 0;
              in_bytes[0] = in_bytes[1];
              in_bytes[1] = in_bytes[2];
              in_bytes[2] = c;
              flags |= INVALID;
          }
          break; // break the while loop even though we have only checked for one package.
          // Todo: Why not flush the buffer? We could have more in the RX buffer we haven't looked at!?
      }
    }

#ifndef DISABLE_ENDSTOPS
    /*
     * Todo: This looks like endstop switches to me. However, the endstops are being used to indicate a fault
     * and the fault stops the the system and goes to center. Why would that make any sense at all? I hit
     * the endstop, just don't move any further and ignore the commant unless you have to move away from
     * the endstop.
     * This only makes sense if these are actual fault pins. But there is no hint as to what these represent.
     */
    // test fault pins
    if(!digitalRead(PORT_FAULT_PIN)) {
        stop_port();
        flags |= PORT_PIN_FAULT;
    } else
      flags &= ~PORT_PIN_FAULT;

    if(!digitalRead(STBD_FAULT_PIN)) {
        stop_starboard();
        flags |= STARBOARD_PIN_FAULT;
    } else
      flags &= ~STARBOARD_PIN_FAULT;
#endif

#ifndef DISABLE_RUDDER_SENSE
/*
 * This section is important as it checks for min/max values and allows motion or not.
 * Observation: 0 to 2000 rudder value with 1000 being the center is actually the PWM value for the h-bridge
 * similar to an RC_Servo.
 * If at 1000, the rudder won't move. It's not going back to center. It just won't move any further.
 * If at 1500, it moves slowly towards one side and if at 500 it moves slowly to the other.
 * I wonder where the deadband is implemented because I can't see it here. PyPolit?
 * Todo: Correct my own comments to represent this new fact (PWM from 0 to 2000 with 1000 being no PWM).
 */
    if (millis() - last_loop_rudder_millis > 100)
    {
      uint16_t v = TakeRudder();
      // if not positive, then rudder feedback has negative gain (reversed)
      uint8_t pos = rudder_min < rudder_max;
      
      if((pos && v < rudder_min) || (!pos && v > rudder_min)) {
          stop_starboard();
          flags |= MIN_RUDDER_FAULT;
      } else
          flags &= ~MIN_RUDDER_FAULT;
      if((pos && v > rudder_max) || (!pos && v < rudder_max)) {
          stop_port();
          flags |= MAX_RUDDER_FAULT;
      } else
          flags &= ~MAX_RUDDER_FAULT;
          
      last_loop_rudder_millis = millis();
    }
#endif

#ifndef DISABLE_CURRENT_SENSE
    if (millis() - last_loop_current_millis > 2000)
    {
      uint16_t amps = TakeAmps(); 
      if(amps >= max_current) {
          stop();
          flags |= OVERCURRENT_FAULT;
      } else
          flags &= ~OVERCURRENT_FAULT;
      last_loop_current_millis = millis();
    }
#endif

#ifndef DISABLE_VOLTAGE_SENSE
    /* 
     * Checks for BADVOLTAGE_FAULT.
     * Todo:  reimpplement a threashold for number of samples that need to be bad before raising BADVOLTAGE.
     *        However, a low pass filter should be enough for the most part. The result should be similar.
     */
    if (millis() - last_loop_voltage_millis > 2000)
    {
      uint16_t volts = TakeVolts();
      // voltage must be between min and max voltage
      if(volts <= (uint16_t)(VIN_MIN) || volts >= (uint16_t)(VIN_MAX)) {
          stop();
          flags |= BADVOLTAGE_FAULT;
      } else
          flags &= ~BADVOLTAGE_FAULT;
  
      last_loop_voltage_millis = millis();
    }
#endif

#ifndef DISABLE_TEMP_SENSE
    /* 
     * Checks for OVERTEMPERATURE.
     * Todo:  Cannot be executed every single time. Arduino not fast enough, apparently.
     */
    if (millis() - last_loop_temperature_millis > 4000)
    {
      uint16_t controller_temp = TakeTemp(CONTROLLER_TEMP);
      uint16_t motor_temp = TakeTemp(MOTOR_TEMP);
      if(controller_temp >= max_controller_temp || motor_temp > max_motor_temp) {
          stop();
          flags |= OVERTEMP_FAULT;
      } else
          flags &= ~OVERTEMP_FAULT;
      last_loop_rudder_millis = millis();
    }
    /*
     * Again, here we reset the controller. Who knows about this? Will PyPilot be notified about the reset?
     * Todo: Investigate if resetting the controller actually makes sense. Restarting will not lower temperature
     */
    /*if(controller_temp > 11000) {
        stop();
        asm volatile ("ijmp" ::"z" (0x0000)); // attempt soft reset
    }*/
#endif

/*
 * The next section sends one byte back to PyPilot. This byte is defined by out_sync_b.
 * Todo: where is this defined and what does it do?
 */
    // output 1 byte
    switch(out_sync_b) {
    case 0:
        // match output rate to input rate
        if(serialin < 4)
            return;

        uint16_t v;
        uint8_t code;

        //  flags C R V C R ct C R mt flags  C  R  V  C  R EE  C  R mct flags  C  R  V  C  R  EE  C  R rr flags  C  R  V  C  R EE  C  R cc  C  R vc
        //  0     1 2 3 4 5  6 7 8  9    10 11 12 13 14 15 16 17 18  19    20 21 22 23 24 25  26 27 28 29    30 31 32 33 34 35 36 37 38 39 40 41 42
        switch(out_sync_pos++) {
        case 0: case 10: case 20: case 30:
#ifdef LOW_CURRENT
                flags |= CURRENT_RANGE;
#endif
            v = flags;
            code = FLAGS_CODE;
            break;
        case 1: case 4: case 7: case 11: case 14: case 17: case 21: case 24: case 27: case 31: case 34: case 37: case 40:
            v = TakeAmps();
            code = CURRENT_CODE;
            serialin-=4; // fix current output rate to input rate
            break;
        case 2: case 5: case 8: case 12: case 15: case 18: case 22: case 25: case 28: case 32: case 35: case 38: case 41:
            v = TakeRudder();
            code = RUDDER_SENSE_CODE;
            break;
        case 3: case 13: case 23: case 33:
            v = TakeVolts();
            code = VOLTAGE_CODE;
            break;
        case 6:
            v = TakeTemp(CONTROLLER_TEMP);
            code = CONTROLLER_TEMP_CODE;
            break;
        case 9:
            v = TakeTemp(MOTOR_TEMP);
            code = MOTOR_TEMP_CODE;
            break;
        case 16: case 26: case 36: /* eeprom reads */
            if(eeprom_read_addr != eeprom_read_end) {
                uint8_t value;
                if(eeprom_read_8(eeprom_read_addr, value)) {
                    v = value << 8 | eeprom_read_addr;
                    eeprom_read_addr++;
                    code = EEPROM_VALUE_CODE;
                    out_sync_pos--; // fast eeprom read
                    break;
                }
                eeprom_read_addr++; // skip for now
            }
            return;
        default:
            return;
        }

        crcbytes[0] = code;
        crcbytes[1] = v;
        crcbytes[2] = v>>8;
        // fall through
    case 1: case 2:
        // write next
        Serial.write(crcbytes[out_sync_b]);
        out_sync_b++;
        break;
    case 3:
        // write crc of sync byte plus bytes transmitted
        Serial.write(crc8(crcbytes, 3));
        out_sync_b = 0;
        break;
    }
}

/*
 ******************************** EEPROM FUNCTIONS *************************************
 */
// we need these to be atomic for 16 bytes
uint16_t eeprom_read_16(int address)
{
    // ensure atomic update of 16 bits with 3 banks
    uint16_t v[3];
    for(int i=0; i<3; i++) {
        int addr = i*256 + address;
        v[i] = eeprom_read_word((uint16_t*)addr);
        //eeprom_read_byte((unsigned char *)addr) | eeprom_read_byte((unsigned char *)addr+1)<<8;
    }

    if(v[1] == v[2])
        return v[2];

    return v[0];
}

void eeprom_update_16(int address, uint16_t value)
{
    // write in 3 locations
    for(int i=0; i<3; i++) {
        int addr = i*256 + address;
        eeprom_update_word((uint16_t*)addr, value);
//        eeprom_update_byte((unsigned char*)addr, value&0xff);
//        eeprom_update_byte((unsigned char*)addr+1, value>>8);
    }
}

uint8_t eeprom_read_8(int address, uint8_t &value)
{
    static uint8_t lastvalue, lastaddress=255;
    if(address & 1) { // odd
        if(address == lastaddress+1) {
            value = lastvalue;
            return 1;
        } else
            return 0;
    }

    // even
    uint16_t v = eeprom_read_16(address);
    value = v&0xff;
    lastvalue = v >> 8;
    lastaddress = address;
    return 1;
}

void eeprom_update_8(int address, uint8_t value)
{
    static uint8_t lastvalue, lastaddress=255;
    if(address & 1) { // odd
        if(address == lastaddress+1)
            eeprom_update_16(lastaddress, lastvalue | value<<8);
    } else {
        lastvalue = value;
        lastaddress = address;
    }
}

void debug(char *fmt, ... ){
    char buf[128]; // resulting string limited to 128 chars
    va_list args;
    va_start (args, fmt );
    vsnprintf(buf, 128, fmt, args);
    va_end (args);
    Serial.print(buf);
}
