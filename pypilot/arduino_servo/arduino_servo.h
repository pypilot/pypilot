
class ArduinoServo
{
    enum Telemetry {CURRENT = 1, VOLTAGE = 2, TEMPERATURE = 4, SPEED = 8, POSITION = 16, FLAGS= 32};
    enum Flags {SYNC = 1, FAULTPIN = 2, OVERCURRENT = 4, ENGAUGED = 8};
public:
    ArduinoServo(int _fd);

    bool initialize();
    void command(double command);
    void stop();
    int poll();
    bool fault();
    void max_current(double value);

    double voltage, current;
    int flags;

private:
    void send_value(uint16_t value);
    void raw_command(uint16_t command);
    int in_sync, out_sync;
    int in_sync_count;
    char in_buf[12];
    int in_buf_len;
    int fd;
    double max_current_value;
};
