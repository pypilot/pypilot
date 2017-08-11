#include <stdio.h>
#include <stdlib.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>

#include "arduino_servo.h"

int main(void)
{
    int fd = open("/dev/ttyAMA0", O_RDWR);
    printf("fd\n %d\n", fd);
    if(fd < 0)
        exit(1);

    ArduinoServo servo(fd);
    if(!servo.initialize(38400)) {
        printf("failed\n");
        return 0;
    }
                   
}
