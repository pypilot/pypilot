/* Copyright (C) 2019 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

#include "arduino_servo_eeprom.h"

static uint16_t tobase255(uint16_t x)
{
    // max value is 65024, and 0xff is forbidden in either byte
    uint16_t h = x/255, l = x%255;

    if(h > 255)
        h = 255; // invalid

    return (h<<8) | l;
}

static uint16_t frombase255(uint16_t x)
{
    // max value is 65024, and 0xff is forbidden in either byte
    uint16_t h = (x>>8), l = x&0xff;
    return h*255 + l;
}

static uint16_t tobase255s(int16_t x)
{
    if(x<-32512 || x>32512)
        return 0xffff;
    return tobase255((int32_t)x+32512);
}

static int16_t frombase255s(uint16_t x)
{
    int32_t z = frombase255(x);
    return z  - 32512;
}


arduino_servo_eeprom::arduino_servo_eeprom()
    : initial_read(false)
{
    memset(&local, 0, sizeof local);
    memset(&verified, 0, sizeof verified);
    memcpy(local.signature, "arsv24", 6); // change this if the format changes
}

double arduino_servo_eeprom::get_max_current()
{
    return frombase255(arduino.max_current)/100.0;
}

void arduino_servo_eeprom::set_max_current(double max_current)
{
    local.max_current = tobase255(round(max_current * 100.0));
}

double arduino_servo_eeprom::get_max_controller_temp()
{
    return frombase255(arduino.max_motor_temp)/100.0;
}

void arduino_servo_eeprom::set_max_controller_temp(double max_controller_temp)
{
    local.max_controller_temp = tobase255(round(max_controller_temp * 100.0));
}

double arduino_servo_eeprom::get_max_motor_temp()
{
    return frombase255(arduino.max_motor_temp)/100.0;
}

void arduino_servo_eeprom::set_max_motor_temp(double max_motor_temp)
{
    local.max_motor_temp = tobase255(round(max_motor_temp * 100.0));
}

double arduino_servo_eeprom::get_rudder_range()
{
    return arduino.rudder_range/2.0;
}

void arduino_servo_eeprom::set_rudder_range(double rudder_range)
{
    local.rudder_range = round(rudder_range * 2); // from 0 to 120 in 0.5 increments
}

// store offset as s9.6 fixed point
double arduino_servo_eeprom::get_rudder_offset()
{
    return frombase255s(arduino.rudder_offset)/64.0;
}

void arduino_servo_eeprom::set_rudder_offset(double rudder_offset)
{
    local.rudder_offset =tobase255s(round(rudder_offset * 64));
}

// store rudder scale s12.3 fixed point
double arduino_servo_eeprom::get_rudder_scale()
{
    return frombase255s(arduino.rudder_scale)/8.0;
}

void arduino_servo_eeprom::set_rudder_scale(double rudder_scale)
{
    local.rudder_scale = tobase255s(round(rudder_scale * 8.0));
}

// store nonlinearity from -1 to 1 for s1.14 fixed point
double arduino_servo_eeprom::get_rudder_nonlinearity()
{
    return frombase255s(arduino.rudder_nonlinearity)/16250.0;
}

void arduino_servo_eeprom::set_rudder_nonlinearity(double rudder_nonlinearity)
{
    local.rudder_nonlinearity = tobase255s(round(rudder_nonlinearity * 16250));
}

double arduino_servo_eeprom::get_max_slew_speed()
{
    return arduino.max_slew_speed/254.0*100.0;
}

void arduino_servo_eeprom::set_max_slew_speed(double max_slew_speed)
{
    local.max_slew_speed = round(max_slew_speed * 254/100);
}

double arduino_servo_eeprom::get_max_slew_slow()
{
    return arduino.max_slew_slow/254.0*100.0;
}

void arduino_servo_eeprom::set_max_slew_slow(double max_slew_slow)
{
    local.max_slew_slow = round(max_slew_slow * 254/100);
}

double arduino_servo_eeprom::get_current_factor()
{
    return .8 + arduino.current_factor * .0016;
}

void arduino_servo_eeprom::set_current_factor(double current_factor)
{
    local.current_factor = round((current_factor - .8)/.0016);
}

double arduino_servo_eeprom::get_current_offset()
{
    return (arduino.current_offset-127)/100.0;
}

void arduino_servo_eeprom::set_current_offset(double current_offset)
{
    local.current_offset = 127+round(current_offset*100.0);
}

double arduino_servo_eeprom::get_voltage_factor()
{
    return .8 + arduino.voltage_factor * .0016;
}

void arduino_servo_eeprom::set_voltage_factor(double voltage_factor)
{
    local.voltage_factor = round((voltage_factor - .8)/.0016);
}

double arduino_servo_eeprom::get_voltage_offset()
{
    return (arduino.voltage_offset-127)/100.0;
}

void arduino_servo_eeprom::set_voltage_offset(double voltage_offset)
{
    local.voltage_offset = 127 + round(voltage_offset*100.0);
}

double arduino_servo_eeprom::get_min_speed()
{
    return arduino.min_speed/2.0;
}
     
void arduino_servo_eeprom::set_min_speed(double min_speed)
{
    local.min_speed = round(min_speed*2.0);
}

double arduino_servo_eeprom::get_max_speed()
{
    return arduino.max_speed/2.0;
}
     
void arduino_servo_eeprom::set_max_speed(double max_speed)
{
    local.max_speed = round(max_speed*2.0);
}

// record gain in thousandths
double arduino_servo_eeprom::get_gain()
{
    return frombase255s(arduino.gain)/1000.0;
}

void arduino_servo_eeprom::set_gain(double gain)
{
    local.gain = tobase255s(round(gain*1000.0));
}

int arduino_servo_eeprom::need_read(uint8_t *end)
{
    int is = sizeof verified;
    for(int i=0; i < is; i++) {
        if(!verified[i]) {
            if(end) {
                *end = i;
                while(*end < is && !verified[*end])
                    (*end)++;
                if((*end)&1) // make even
                    (*end)++;
            }
            return i&~1; // always even
        }
    }
    return -1;
}

int arduino_servo_eeprom::need_write()
{
    if(!initial_read) // don't write before reading
        return -1;

    uint8_t *l = (uint8_t*)&local, *a = (uint8_t*)&arduino;
    int ls = sizeof local;
        
    for(int i=0; i < ls; i++)
        if(l[i] != a[i] && verified[i]) { // local and remote data doesn't match
            i&=~1; // always even
            verified[i] = 0; // read this byte again
            verified[i+1] = 0; // read this byte again
            return i;
        }
    return -1;
}

// return true  only once ever if initial eeprom data is read
bool arduino_servo_eeprom::initial()
{
    if(initial_read || need_read() != -1)
        return false;
    initial_read = true; // allow writes now

    // signature failed, discard this data
    if(memcmp(arduino.signature, local.signature, sizeof local.signature)) {
        printf("Arduino EEPROM Signature FAILED!\n");
        return false;
    }

    uint8_t *a = (uint8_t*)&arduino;
    int ls = sizeof arduino;

    // discard if any byte is 0xff.
    // This is invalid data and is also the initial eeprom value
    for(int i=0; i < ls; i++) {
        if(a[i] == 0xff) {
            printf("EEPROM SIGNATURE invalid byte %d\n", i);
            return false;
        }
    }
    printf("EEPROM SIGNATURE ok\n");
    
    return true;
}

void arduino_servo_eeprom::value(uint8_t addr, uint8_t val)
{
    if(addr >= sizeof *this)
        return;
    ((uint8_t *)&arduino)[addr] = val;
    ((uint8_t *)&verified)[addr] = 1;
}
