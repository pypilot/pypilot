#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <errno.h>
#include "arduino_servo.h"

int main(void)
{
    const char device[] = "/dev/ttyAMA0";
    
    int fd = open(device, O_RDWR);
    if(fd < 0) {
        printf("failed to open %s\n", device);
        exit(1);
    }

    int ret = fcntl(fd, F_SETFL, O_NONBLOCK|fcntl(fd, F_GETFL));
    if(ret < 0) {
        printf("nonblocking failed\n");
        exit(1);
    }

    printf("detecting servo on %s ...\n", device);
    ArduinoServo servo(fd);
    //servo.params(1.0, 0, 1, .5, 60, 60, 50, 0, 1, 0, 40, 40, 1, 0, 1, 0, 1, 1, 1);
    while(1) {
        int ret = servo.poll();
        if(ret) {
            printf("servo detected\n");
            exit(0);
        }
        usleep(100000);
    }
}
