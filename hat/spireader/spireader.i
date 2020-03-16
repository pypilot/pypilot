/* File: spireader.i */
%module spireader
%begin %{
    #define SWIG_PYTHON_STRICT_BYTE_CHAR
%}

%{
#include "spireader.h"
%}

class spireader
{
public:
    spireader(int ms, int repeatn);
    virtual ~spireader();

    int open(int portn, int slave, int baud);
    void close();
    unsigned char xfer(unsigned char c, bool block);
};
