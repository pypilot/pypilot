/* File: spireader.i */
%module spireader
%begin %{
    #define SWIG_PYTHON_STRICT_BYTE_CHAR
%}

%{
#include "spireader.h"
%}

%include <stdint.i>
%include <std_vector.i>
%include <exception.i>

%exception {
    try {
        $action
    } catch (const std::exception &e) {
        SWIG_exception(SWIG_RuntimeError, e.what());
    }
}


/* expose std::vector<uint8_t> */
namespace std {
    %template(Uint8Vector) vector<uint8_t>;
}

%include "spireader.h"
