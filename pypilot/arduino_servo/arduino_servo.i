/* File: arduino_servo.i */
%module arduino_servo

%{
#include "arduino_servo.h"
%}

class ArduinoServo
{
    enum {CURRENT = 1, VOLTAGE = 2, TEMPERATURE = 4, SPEED = 8, POSITION = 16};
    enum {SYNC = 1, FAULTPIN = 2, OVERCURRENT = 4, ENGAUGED = 8};

public:
    ArduinoServo(int _fd);

    bool initialize(int baud);
    void command(double command);
    void stop();
    void disengauge();
    void reprogram();
    int poll();
    bool fault();
    void max_values(double current, double arduino_temp);

    double voltage, current;
    int flags;
};
