
class ArduinoServo
{
    enum Telemetry {FLAGS= 1, CURRENT = 2, VOLTAGE = 4, SPEED = 8, POSITION = 16, CONTROLLER_TEMP = 32, MOTOR_TEMP = 64, RUDDER_POS=128};
    enum flags {SYNC=1, OVERTEMP=2, OVERCURRENT=4, ENGAUGED=8, INVALID=16*1, FAULTPIN=16*2};
public:
    ArduinoServo(int _fd);

    bool initialize(int baud);
    void command(double command);
    void stop();
    void disengauge();
    void reprogram();
    int poll();
    bool fault();
    void max_values(double current, double controller_temp, double motor_temp);

    double voltage, current, controller_temp, motor_temp, rudder_pos;
    int flags;

private:
    void send_value(uint8_t command, uint16_t value);
    void raw_command(uint16_t value);
    int process_packet(uint8_t *in_buf);
    int in_sync_count;
    uint8_t in_buf[16];
    int in_buf_len;
    int fd;
    int out_sync;
    double max_current_value, max_controller_temp_value, max_motor_temp_value;
};
