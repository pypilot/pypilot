"""The C++ LineBuffer, built with the system compiler since the SWIG extension
is only built when pypilot is installed."""
import os
import shutil
import subprocess

import pytest

SOURCE = os.path.join(os.path.dirname(__file__), '..', 'pypilot', 'linebuffer')

# reads lines from stdin with LineBuffer and prints the length of each
DRIVER = r'''
#include <stdio.h>
#include <string.h>
#include <poll.h>
#include "linebuffer.h"

int main()
{
    LineBuffer b(0);
    for(;;) {
        const char *line;
        while((line = b.line()))
            printf("%d\n", (int)strlen(line));
        struct pollfd p = {0, POLLIN, 0};
        poll(&p, 1, -1);
        if(!b.recv())
            return 0;
    }
}
'''


@pytest.fixture(scope='module')
def driver(tmp_path_factory):
    compiler = shutil.which('g++') or shutil.which('c++')
    if not compiler:
        pytest.skip('no C++ compiler')
    d = tmp_path_factory.mktemp('linebuffer')
    (d / 'driver.cpp').write_text(DRIVER)
    exe = d / 'driver'
    subprocess.run([compiler, '-O1', '-fsanitize=address', '-I', SOURCE, str(d / 'driver.cpp'),
                    os.path.join(SOURCE, 'linebuffer.cpp'), '-o', str(exe)], check=True)
    return str(exe)


def line_lengths(driver, data):
    out = subprocess.run([driver], input=data, capture_output=True, check=True).stdout.decode()
    return [int(n) for n in out.splitlines() if n.isdigit()], out


def test_short_lines(driver):
    lengths, _ = line_lengths(driver, b'a=1\nbb=2\n')
    assert lengths == [4, 5]  # with their newlines


@pytest.mark.parametrize('size', [16382, 16383, 16384, 16385, 40000, 200000, (1 << 20) - 1])
def test_long_lines(driver, size):
    # pypilot's list of values is one line, and can pass 16 KB
    long_line = b'v' * (size - 1) + b'\n'
    lengths, _ = line_lengths(driver, b'x=1\n' + long_line + b'y=2\n')
    assert lengths == [4, size, 4]


@pytest.mark.parametrize('size', [(1 << 20) - 1, 1 << 20, (1 << 20) + 123, (3 << 20) + 123])
def test_runaway_input_is_dropped(driver, size):
    # input with no newline doesn't grow the buffer without limit
    lengths, out = line_lengths(driver, b'first=1\n' + b'z' * size + b'\nok=1\nlast=2\n')
    assert 'overflow' in out
    assert lengths == [8, 5, 7]  # no suffix of the oversized line may be returned
