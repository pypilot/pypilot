/* File: arduino_servo.i */
%module arduino_servo

%{
#include "arduino_servo.h"
%}

class ArduinoServo
{
    enum Telemetry {FLAGS= 1, CURRENT = 2, VOLTAGE = 4, SPEED = 8, POSITION = 16, CONTROLLER_TEMP = 32, MOTOR_TEMP = 64, RUDDER = 128, MAX_CURRENT = 256, MAX_CONTROLLER_TEMP = 512, MAX_MOTOR_TEMP = 1024, RUDDER_RANGE = 2048, MAX_SLEW = 4096, CURRENT_CORRECTION = 8192, VOLTAGE_CORRECTION = 16384};
    enum {SYNC=1, OVERTEMP_FAULT=2, OVERCURRENT_FAULT=4, ENGAGED=8, INVALID=16*1, PORT_PIN_FAULT=16*2, STARBOARD_PIN_FAULT=16*4};

public:
    ArduinoServo(int _fd);

    void command(double command);
    void reset();
    void disengage();
    void reprogram();
    int poll();
    bool fault();
    void params(double _raw_max_current, double _rudder_min, double _rudder_max, double _max_current, double _max_controller_temp, double _max_motor_temp, double _rudder_range, double _rudder_offset, double _rudder_scale, double _rudder_nonlinearity, double _max_slew_speed, double _max_slew_slow, double _current_factor, double _current_offset, double _voltage_factor, double _voltage_offset, double _min_speed, double _max_speed, double _gain, double _clutch_pwm);

    double voltage, current, controller_temp, motor_temp, rudder;

    double max_current, max_controller_temp, max_motor_temp;
    double rudder_range, rudder_offset, rudder_scale, rudder_nonlinearity;
    double max_slew_speed, max_slew_slow;
    double current_factor, current_offset, voltage_factor, voltage_offset;
    double min_speed, max_speed;
    double gain;

    int flags;
};
