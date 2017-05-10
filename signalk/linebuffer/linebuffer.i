/* File: linebuffer.i */
%module linebuffer

%{
#include "linebuffer.h"
%}

class LineBuffer {
public:
    LineBuffer(int _fd);

    bool next();
    bool next_nmea();
    const char *readline_nmea();
    const char *line();
    bool recv();
};
