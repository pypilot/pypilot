/* Copyright (C) 2021 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

#include <time.h>

class spireader
{
public:
    spireader(int ms, int repeatn);
    virtual ~spireader();

    int open(int portn, int slave, int baud);
    void close();
    unsigned char xfer(unsigned char c, bool block);

private:
    struct timespec ts;
    int repeat;
    int spifd;
    int port;
};
