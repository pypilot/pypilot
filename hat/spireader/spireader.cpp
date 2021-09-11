/* Copyright (C) 2021 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

#include <stdio.h>
#include <unistd.h>
#include <wiringPiSPI.h>

#include "spireader.h"

spireader::spireader(int ms, int repeatn)
    : repeat(repeatn)
{
    ts.tv_sec = 0;
    ts.tv_nsec = ms * 1e6;
}

int spireader::open(int portn, int slave, int baud)
{
    port = slave;
    spifd = wiringPiSPISetup(slave, baud);
    return spifd;
}

spireader::~spireader()
{
    close();
}

void spireader::close()
{
    ::close(spifd);
}

unsigned char spireader::xfer(unsigned char c, bool block)
{
    if(block)
        for(int i=0; i < repeat; i++) {
            wiringPiSPIDataRW(port, &c, 1);
            if(c)
                return c;
            nanosleep(&ts, 0);
        }
    
    wiringPiSPIDataRW(port, &c, 1);
    return c;
}
