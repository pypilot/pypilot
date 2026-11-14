/* Copyright (C) 2017 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

#include <vector>

class LineBuffer {
public:
    LineBuffer(int _fd);

    const char *line();
    const char *line_nmea();
    bool recv();

    const char *readline_nmea();
private:
    bool next_nmea();
#if 0    
    const char *readline();
    bool next();
#endif    
    bool readline_buf_nmea();
    int readline_buf();
    bool grow();

    int fd;
    int b, pos, len;
    bool discarding;
    // two buffers of the same size, doubled as needed for longer lines
    // (pypilot's list of values is one line) up to a limit, for input with
    // no newlines
    std::vector<char> buf[2];
};
