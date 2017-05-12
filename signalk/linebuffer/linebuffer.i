/* File: linebuffer.i */
%module linebuffer

%{
#include "linebuffer.h"
%}

class LineBuffer {
public:
    LineBuffer(int _fd);

    const char *line();
    const char *line_nmea();
    bool recv();
//    const char *readline();
    const char *readline_nmea();
};
