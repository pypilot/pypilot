
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
    void max_values(double current, double controller_temp, double motor_temp, double min_rudder, double max_rudder, double max_slew_speed, double max_slew_slow);
    void params(double current_factor, double current_offset, double voltage_factor, double voltage_offset);

    double voltage, current, controller_temp, motor_temp, rudder;
    double current_factor, current_offset, voltage_factor, voltage_offset;
    int flags;

private:
    void send_value(uint8_t command, uint16_t value);
    void raw_command(uint16_t value);
    int process_packet(uint8_t *in_buf);
    int in_sync_count;
    uint8_t in_buf[256];
    int in_buf_len;
    int fd;
    int out_sync;
    double max_current_value, max_controller_temp_value, max_motor_temp_value;
    double min_rudder_value, max_rudder_value;
    double max_slew_speed_value, max_slew_slow_value;
    int params_set;
};
