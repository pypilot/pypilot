/* Copyright (C) 2017 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

class LineBuffer {
public:
    LineBuffer(int _fd);

    bool next();
    bool next_nmea();
    const char *readline_nmea();
    const char *readline();
    const char *line();
    bool recv();

private:
    bool readline_buf_nmea();
    int readline_buf();

    int fd;
    int b, pos, len;
    char buf[2][4096];
};
