/* Copyright (C) 2017 Sean D'Epagnier <seandepagnier@gmail.com>
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

#include "arduino_servo.h"

enum commands {COMMAND_CODE = 0xc7, STOP_CODE = 0xe7, MAX_CURRENT_CODE = 0x1e, MAX_ARDUINO_TEMP_CODE = 0xa7, REPROGRAM_CODE = 0x19};
enum results {CURRENT_CODE = 0x1c, VOLTAGE_CODE = 0xb3, ARDUINO_TEMP_CODE=0xf9, FLAGS_CODE = 0x41};

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

ArduinoServo::ArduinoServo(int _fd)
    : fd(_fd)
{
    in_sync_count = 0;
    out_sync = 0;
    in_buf_len = 0;
    max_current_value = 0;

    flags = 0;
}

bool ArduinoServo::initialize(int baud)
{
    int cnt = 0;
    bool data = false;
    while (flags & OVERCURRENT || !(flags & SYNC)) {
        stop();
        if(poll()>0) {
            while(poll());
            data = true;
        } else
            usleep(1e6 * 120 / baud);
        cnt++;
        if(cnt >= 400 && !data) {
            printf("arduino servo fail no data\n");
            return false;
        }
        if(cnt == 1000) {
            printf("arduino servo fail sync\n");
            return false;
        }
    }
    return true;
}

void ArduinoServo::command(double command)
{    
    command = fmin(fmax(command, -1), 1);
    raw_command((command+1)*1000);
}

int ArduinoServo::process_packet(uint8_t *in_buf)
{
    uint16_t value = in_buf[1] + (in_buf[2]<<8);
    switch(in_buf[0]) {
    case CURRENT_CODE:
        current = value / 100.0;
        return CURRENT;
    case VOLTAGE_CODE:
        voltage = value / 100.0;
        return VOLTAGE;
    case ARDUINO_TEMP_CODE:
        arduino_temp = (int16_t)value / 100.0;
        return ARDUINO_TEMP;
    case FLAGS_CODE:
        flags = value;
        return FLAGS;
    }
    return 0;
}    

int ArduinoServo::poll()
{
    if(in_buf_len < 4) {
        int c = read(fd, in_buf + in_buf_len, sizeof in_buf - in_buf_len);
        if(c<=0) // todo: support failure if the device is unplugged
            return 0;
        in_buf_len += c;
        if(in_buf_len < 4)
            return 0;
    }

    int ret = 0;
    while(in_buf_len >= 4) {
        uint8_t crc = crc8(in_buf, 3);
        if(crc == in_buf[3]) { // valid packet
            if(in_sync_count >= 3)
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

    return ret;
}

bool ArduinoServo::fault()
{
    return flags & (FAULTPIN | OVERCURRENT);
}

void ArduinoServo::max_values(double current, double arduino_temp)
{
    max_current_value = fmin(10, fmax(0, current));
    max_arduino_temp_value = fmin(80, fmax(30, arduino_temp));
}

void ArduinoServo::send_value(uint8_t command, uint16_t value)
{
    uint8_t code[3] = {command, (uint8_t)(value&0xff), (uint8_t)((value>>8)&0xff)};
    uint8_t b[4] = {code[0], code[1], code[2], crc8(code, 3)};
    write(fd, b, 4);
}

void ArduinoServo::raw_command(uint16_t value)
{
    // send max current and temp occasionally
    switch(out_sync++) {
    case 0:
        send_value(MAX_CURRENT_CODE, max_current_value*100);
        break;
    case 4:
        send_value(MAX_ARDUINO_TEMP_CODE, max_arduino_temp_value*100);
        break;
    case 7:
        out_sync = 0;
    }
    send_value(COMMAND_CODE, value);
        
}

void ArduinoServo::stop()
{
    send_value(STOP_CODE, 0);
}
