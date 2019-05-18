/* Copyright (C) 2019 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

#include <stdint.h>
#include <unistd.h>
#include <math.h>

#include <stdio.h>
#include <errno.h>

#include <sys/time.h>

#include "arduino_servo.h"

enum commands {COMMAND_CODE=0xc7, RESET_CODE=0xe7, MAX_CURRENT_CODE=0x1e, MAX_CONTROLLER_TEMP_CODE=0xa4, MAX_MOTOR_TEMP_CODE=0x5a, RUDDER_RANGE_CODE=0xb6, REPROGRAM_CODE=0x19, DISENGAGE_CODE=0x68, MAX_SLEW_CODE=0x71, EEPROM_READ_CODE=0x91, EEPROM_WRITE_CODE=0x53};

enum results {CURRENT_CODE=0x1c, VOLTAGE_CODE=0xb3, CONTROLLER_TEMP_CODE=0xf9, MOTOR_TEMP_CODE=0x48, RUDDER_SENSE_CODE=0xa7, FLAGS_CODE=0x8f, EEPROM_VALUE_CODE=0x9a};

const unsigned char crc8_table[256]
= {
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97,
    0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
    0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
    0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
    0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11,
    0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52,
    0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
    0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
    0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
    0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9,
    0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C,
    0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
    0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
    0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
    0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED,
    0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE,
    0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
    0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
    0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
    0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28,
    0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0,
    0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
    0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
    0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
    0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56,
    0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15,
    0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC
};

static inline uint8_t crc8_byte(uint8_t old_crc, uint8_t byte){
    return crc8_table[old_crc ^ byte];
}

static inline uint8_t crc8_with_init(uint8_t init_value, uint8_t *pcBlock, uint8_t len)
{
    uint8_t crc = init_value;
    while (len--)
        crc = crc8_byte(crc, *pcBlock++);
    return crc;
}

static uint8_t crc8(uint8_t *pcBlock, uint8_t len) {
    return crc8_with_init(0xFF, pcBlock, len);
}


ArduinoServo::ArduinoServo(int _fd, int _baud)
    : fd(_fd), baud(_baud)
{
    in_sync_count = 0;
    out_sync = 0;
    in_buf_len = 0;
    max_current = 0;
    params_set = 0;
    flags = 0;

    // force unsync
    uint8_t reset_code[] = {0xff, 0xff, 0xff, 0xff};
    write(fd, reset_code, sizeof reset_code);
    // flush device data
//    while(read(fd, in_buf, in_buf_len) > 0);
    nosync_count = 0;
    nosync_data = 0;
}

void ArduinoServo::command(double command)
{    
    command = fmin(fmax(command, -1), 1);
    raw_command((command+1)*1000);
}

int ArduinoServo::process_packet(uint8_t *in_buf)
{    
    if(packet_count < 255)
        packet_count++;
    uint16_t value = in_buf[1] + (in_buf[2]<<8);
//    printf("buf %x %x %x\n", in_buf[0], in_buf[1], in_buf[2]);
    switch(in_buf[0]) {
    case CURRENT_CODE:
        current = value / 100.0;
        //printf("servo current  %f\n", current);
        return CURRENT;
    case VOLTAGE_CODE:
        voltage = value / 100.0;
        //printf("servo voltage  %f\n", voltage);
        return VOLTAGE;
    case CONTROLLER_TEMP_CODE:
        controller_temp = (int16_t)value / 100.0;
        //printf("servo temp  %f\n", controller_temp);
        return CONTROLLER_TEMP;
    case MOTOR_TEMP_CODE:
        motor_temp = (int16_t)value / 100.0;
        return MOTOR_TEMP;
    case RUDDER_SENSE_CODE:
        if(value == 65535)
            rudder = NAN;
        else
            rudder = (uint16_t)value / 727.0 - 45.0; // nominal range +- 45
        return RUDDER;
    case FLAGS_CODE:
        flags = value;
        if(flags & INVALID)
            printf("servo received invalid packet (check serial connection)\n");
        return FLAGS;
        
    case EEPROM_VALUE_CODE:
    {
        //printf("EEPROM VALUE %d %d\n", in_buf[1], in_buf[2]);
        uint8_t addr = in_buf[1], val = in_buf[2];
        static uint8_t lastaddr, lastvalue;
        if(addr&1) {
            if(addr == lastaddr+1) {
                eeprom.value(lastaddr, lastvalue);
                eeprom.value(addr, val);
            }
        } else {
            lastaddr = addr;
            lastvalue = val;
        }

        // only report eeprom on initial read for all data
        if(eeprom.initial()) {
            max_current = eeprom.get_max_current();
            max_controller_temp = eeprom.get_max_controller_temp();
            max_motor_temp = eeprom.get_max_motor_temp();
            rudder_range = eeprom.get_rudder_range();
            rudder_offset = eeprom.get_rudder_offset();
            rudder_scale = eeprom.get_rudder_scale();
            rudder_nonlinearity = eeprom.get_rudder_nonlinearity();
            max_slew_speed = eeprom.get_max_slew_speed();
            max_slew_slow = eeprom.get_max_slew_slow();
            current_factor = eeprom.get_current_factor();
            current_offset = eeprom.get_current_offset();
            voltage_factor = eeprom.get_voltage_factor();
            voltage_offset = eeprom.get_voltage_offset();
            min_motor_speed = eeprom.get_min_motor_speed();
            max_motor_speed = eeprom.get_max_motor_speed();
            gain = eeprom.get_gain();

            // validate ranges
            params(max_current, max_controller_temp, max_motor_temp, rudder_range, rudder_offset, rudder_scale, rudder_nonlinearity, max_slew_speed, max_slew_slow, current_factor, current_offset, voltage_factor, voltage_offset, min_motor_speed, max_motor_speed, gain);
            return EEPROM;
        } else if(!eeprom.initial_read) {
            // if we got an eeprom value, but did not get the initial read,
            // send a lot of disengage commands to speed up communication speed which
            // will complete reading eeprom faster
            for(int i=0; i<16; i++)
                disengauge();
        }
    }
    }
    return 0;
}

int ArduinoServo::poll()
{
    if (!(flags & SYNC)) {
#if 0
        gettimeofday(&tv, 0);
        double dt = 0;
        while(dt < .01) { 
            gettimeofday(&tv2, 0);
            double dt = tv2.tv_sec - tv.tv_sec + (tv2.tv_usec - tv.tv_usec) / 1e6.
        }
#endif
        raw_command(1000); // ensure we set the temp limits as well here
        nosync_count++;
        if(nosync_count >= 400 && !nosync_data) {
            printf("arduino servo fail no data\n");
            return -1;
        }
        if(nosync_count >= 1000) {
            printf("arduino servo fail sync\n");
            return -1;
        }
    } else {
        // reset incase we need to reinitialize
        nosync_count = 0;
        nosync_data = 0;
    }

    if(in_buf_len < 4) {
        int c;
        for(;;) {
            int cnt = sizeof in_buf - in_buf_len;
            c = read(fd, in_buf + in_buf_len, cnt);
            if(c < cnt)
                break;
            in_buf_len = 0;
            printf("arduino servo buffer overflow\n");
        }
        if(c<0) {
            if(errno != EAGAIN)
                return -1;
        }
        in_buf_len += c;
        if(in_buf_len < 4)
            return 0;
    }

    int ret = 0;
    while(in_buf_len >= 4) {
        uint8_t crc = crc8(in_buf, 3);
#if 0
        static int cnt;
        struct timeval tv;
        gettimeofday(&tv, 0);
        printf("input %d %ld:%ld %x %x %x %x %x %d\n", cnt++, tv.tv_sec, tv.tv_usec, in_buf[0], in_buf[1], in_buf[2], in_buf[3], crc, in_buf_len);
#endif
        if(crc == in_buf[3]) { // valid packet
            if(in_sync_count >= 2)
                ret |= process_packet(in_buf);
            else
                in_sync_count++;
            in_buf_len-=4;
            for(int i=0; i<in_buf_len; i++)
                in_buf[i] = in_buf[i+4];
        } else {
            // invalid packet, shift by 1 byte
            in_sync_count = 0;
            in_buf_len--;
            for(int i=0; i<in_buf_len; i++)
                in_buf[i] = in_buf[i+1];
        }
    }

    if (flags & SYNC)
        return ret;

    if (ret)
        nosync_data = 1;
    
    return 0;
}

bool ArduinoServo::fault()
{
    return flags & OVERCURRENT;
}

void ArduinoServo::params(double _max_current, double _max_controller_temp, double _max_motor_temp, double _rudder_range, double _rudder_offset, double _rudder_scale, double _rudder_nonlinearity, double _max_slew_speed, double _max_slew_slow, double _current_factor, double _current_offset, double _voltage_factor, double _voltage_offset, double _min_motor_speed, double _max_motor_speed, double _gain)
{
    max_current = fmin(40, fmax(0, _max_current));
    eeprom.set_max_current(max_current);

    max_controller_temp = fmin(80, fmax(30, _max_controller_temp));
    eeprom.set_max_controller_temp(max_controller_temp);

    max_motor_temp = fmin(80, fmax(30, _max_motor_temp));
    eeprom.set_max_motor_temp(max_motor_temp);

    rudder_range = fmin(120, fmax(0, _rudder_range));
    eeprom.set_rudder_range(rudder_range);

    rudder_offset = fmin(60, fmax(-60, _rudder_offset));
    eeprom.set_rudder_offset(rudder_offset);

    rudder_scale = fmin(400, fmax(-400, _rudder_scale));
    eeprom.set_rudder_scale(rudder_scale);

    rudder_nonlinearity = fmin(2, fmax(-2, _rudder_nonlinearity));
    eeprom.set_rudder_nonlinearity(rudder_nonlinearity);

    max_slew_speed = fmin(100, fmax(0, _max_slew_speed));
    eeprom.set_max_slew_speed(max_slew_speed);

    max_slew_slow = fmin(100, fmax(0, _max_slew_slow));
    eeprom.set_max_slew_slow(max_slew_slow);

    current_factor = fmin(1.2, fmax(.8, _current_factor));
    eeprom.set_current_factor(current_factor);

    current_offset = fmin(1.2, fmax(-1.2, _current_offset));
    eeprom.set_current_offset(current_offset);

    voltage_factor = fmin(1.2, fmax(.8, _voltage_factor));
    eeprom.set_voltage_factor(voltage_factor);

    voltage_offset = fmin(1.2, fmax(-1.2, _voltage_offset));
    eeprom.set_voltage_offset(voltage_offset);

    min_motor_speed = fmin(1, fmax(0, _min_motor_speed));
    eeprom.set_min_motor_speed(min_motor_speed);
    
    max_motor_speed = fmin(1, fmax(0, _max_motor_speed));
    eeprom.set_max_motor_speed(max_motor_speed);

    gain = fmin(10, fmax(-10, _gain));
    if(gain < 0)
        gain = fmin(gain, -.5);
    else
        gain = fmax(gain, .5);
            
    eeprom.set_gain(gain);

    params_set = 1;
}

void ArduinoServo::send_value(uint8_t command, uint16_t value)
{
    uint8_t code[4] = {command, (uint8_t)(value&0xff), (uint8_t)((value>>8)&0xff), 0};
    code[3] = crc8(code, 3);
#if 0
    struct timeval tv;
    gettimeofday(&tv, 0);
    printf("output %ld:%ld %x %x %x %x\n", tv.tv_sec, tv.tv_usec, code[0], code[1], code[2], code[3]);
#endif
    write(fd, code, 4);
}

static double quad_sub(double a, double b, double c, double m)
{
    if(fabs(a) < .0001) // in linear case
        return -c/b;

    double dis = b*b - 4*a*c;
    //printf("quadsub %f %f %f %f, %f\n", a, b,c, m, dis);
    if(dis < 0)
        return -1; // invalid in this case
    return (-b + m*sqrt(dis)) / (2*a);
}

void ArduinoServo::send_params()
{
    // send parameters occasionally, but only after parameters have been
    // initialized by the upper level
    if (params_set)
        switch(out_sync) {
        case 0: case 8: case 16:
            send_value(MAX_CURRENT_CODE, eeprom.local.max_current);
            break;
        case 4:
            send_value(MAX_CONTROLLER_TEMP_CODE, eeprom.local.max_controller_temp);
            break;
        case 6:
            send_value(MAX_MOTOR_TEMP_CODE, eeprom.local.max_motor_temp);
            break;
        case 12:
        {
            //  nonlinearity * raw**2 + scale * raw + offset - rudder_range
            double min = 1, max = 0;
            for(int i=0; i<4; i++) {
                double x = quad_sub(rudder_nonlinearity, rudder_scale, rudder_offset + (i < 2 ? 1 : -1) * rudder_range, i%2 ? 1 : -1);
                //printf("suba %d %f\n", i , x);
                if(x > -.5 && x < .5)
                    min = fmin(min, x), max = fmax(max, x);
            }

            if(min > max) // invalid
                min = -.5, max = .5; // allow movement, maybe not calibrated?
            //printf("min/max %f %f\n", min, max);

            send_value(RUDDER_RANGE_CODE,
                       ((int)round((min+.5)*255) & 0xff) << 8 |
                       ((int)round((max+.5)*255) & 0xff));
        } break;
        case 18:
            send_value(MAX_SLEW_CODE,
                       eeprom.local.max_slew_slow << 8 |
                       eeprom.local.max_slew_speed);
            break;
#if 1
        case 20:
        {
            uint8_t end;
            int addr = eeprom.need_read(&end);
            if(addr >= 0 && end > addr) {
                send_value(EEPROM_READ_CODE, addr | end<<8);
                //printf("EEPROM_READ %d %d\n", addr, end);
            }
        } break;
        case 22:
        {
            int addr = eeprom.need_write();
            if(addr >= 0) {

#if 0
                printf("\nEEPROM local:\n");
                for(unsigned int i=0; i< sizeof eeprom.local; i+=2) {
                    printf("%d %x %x\n", i, ((uint8_t*)&eeprom.local)[i], ((uint8_t*)&eeprom.local)[i+1]);
                }
#endif
                
                //printf("EEPROM_WRITE %d %d %d\n", addr, eeprom.data(addr), eeprom.data(addr+1));
                // send two packets, always write 16 bits atomically
                send_value(EEPROM_WRITE_CODE, addr | eeprom.data(addr)<<8);
                addr++;
                send_value(EEPROM_WRITE_CODE, addr | eeprom.data(addr)<<8);
            }
        }
#endif
    }

    if(++out_sync == 23)
        out_sync = 0;
}

void ArduinoServo::raw_command(uint16_t value)
{
    send_params();
    send_value(COMMAND_CODE, value);
}

void ArduinoServo::reset()
{
    send_value(RESET_CODE, 0);
}

void ArduinoServo::disengauge()
{
    send_params();
    send_value(DISENGAGE_CODE, 0);
}

void ArduinoServo::reprogram()
{
    send_value(REPROGRAM_CODE, 0);
}
