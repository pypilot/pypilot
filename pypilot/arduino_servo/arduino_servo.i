/* File: arduino_servo.i */
%module arduino_servo

%{
#include "arduino_servo.h"
%}

class ArduinoServo
{
    enum Telemetry {FLAGS= 1, CURRENT = 2, VOLTAGE = 4, SPEED = 8, POSITION = 16, CONTROLLER_TEMP = 32, MOTOR_TEMP = 64, RUDDER = 128, CURRENT_CORRECTION = 256, VOLTAGE_CORRECTION = 512};
    enum {SYNC=1, OVERTEMP=2, OVERCURRENT=4, ENGAGED=8, INVALID=16*1, FWD_FAULTPIN=16*2, REV_FAULTPIN=16*4};

public:
    ArduinoServo(int _fd);

    bool initialize(int baud);
    void command(double command);
    void reset();
    void disengauge();
    void reprogram();
    int poll();
    bool fault();
    void max_values(double current, double arduino_temp, double motor_temp, double min_rudder, double max_rudder, double max_slew_speed, double max_slew_slow);
    void params(double _current_factor, double _current_offset, double _voltage_factor, double _voltage_offset);

    double voltage, current, controller_temp, motor_temp, rudder;
    double current_factor, current_offset, voltage_factor, voltage_offset;
    int flags;
};
