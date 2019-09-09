/* Copyright (C) 2018 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

#include "arduino_servo_eeprom.h"

class ArduinoServo
{
    enum Telemetry {FLAGS= 1, CURRENT = 2, VOLTAGE = 4, SPEED = 8, POSITION = 16, CONTROLLER_TEMP = 32, MOTOR_TEMP = 64, RUDDER = 128, EEPROM = 256};
    enum {SYNC=1, OVERTEMP=2, OVERCURRENT=4, ENGAGED=8, INVALID=16*1, FWD_FAULTPIN=16*2, REV_FAULTPIN=16*4};
public:
    ArduinoServo(int _fd, int _baud);

    void command(double command);
    void reset();
    void disengage();
    void reprogram();
    int poll();
    bool fault();
    void params(double _max_current, double _max_controller_temp, double _max_motor_temp, double _rudder_range, double _rudder_offset, double _rudder_scale, double _rudder_nonlinearity, double _rudder_min, double _rudder_max, double _max_slew_speed, double _max_slew_slow, double _current_factor, double _current_offset, double _voltage_factor, double _voltage_offset, double _min_motor_speed, double _max_motor_speed, double _gain);

    double voltage, current, controller_temp, motor_temp, rudder;

    double max_current, max_controller_temp, max_motor_temp;
    double rudder_min, rudder_max;
    double rudder_range, rudder_offset, rudder_scale, rudder_nonlinearity;
    double max_slew_speed, max_slew_slow;
    double current_factor, current_offset, voltage_factor, voltage_offset;

    double min_motor_speed, max_motor_speed;
    double gain;
    
    int flags;

private:
    void send_value(uint8_t command, uint16_t value);
    void send_params();
    void raw_command(uint16_t value);
    int process_packet(uint8_t *in_buf);
    int in_sync_count;
    uint8_t in_buf[256];
    int in_buf_len;
    int fd, baud;
    int out_sync;
    int params_set;
    int packet_count;

    int nosync_count, nosync_data;

    arduino_servo_eeprom eeprom;
    int eeprom_read;
};
