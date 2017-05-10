/* Copyright (C) 2017 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "linebuffer.h"

// implement line buffering and
// nmea checksum test in c++ for efficiency

LineBuffer::LineBuffer(int _fd) 
    : fd(_fd)
{
    pos = len = b = 0;
}

bool LineBuffer::next()
{
    if(readline_buf() || (recv() && readline_buf()))
        return true;
    if(len == sizeof buf[0]) {
        printf("overflow!!!!\n");
        len = 0;
    }
    return false;
}

bool LineBuffer::next_nmea()
{
    if(readline_buf_nmea() || (recv() && readline_buf_nmea()))
        return true;
    if(len == sizeof buf[0]) {
        printf("overflow!!!!\n");
        len = 0;
    }
    return false;
}

const char *LineBuffer::readline_nmea()
{
    if(next_nmea())
        return buf[!b];
    return NULL;
}

const char *LineBuffer::readline()
{
    if(next())
        return buf[!b];
    return NULL;
}

const char *LineBuffer::line()
{
    return buf[!b];
}

bool LineBuffer::recv()
{
    int c = read(fd, buf[b] + len, sizeof buf[0] - len);
    if(c <= 0)
        return false;
    len += c;
    return true;
}

static int nmea_cksum(const char *buf, int len)
{
    int value = 0;
    for(int i=0; i<len; i++)
        value ^= buf[i];

    return value;
}

static bool check_nmea_cksum(const char *buf, int len)
{
    if(len < 4 || buf[0] != '$')
        return false;
    int cksum = strtol(buf+len-2, 0, 16);
    return cksum == nmea_cksum(buf+1, len-4);
}

// return true if a valid nmea line is in buf[!b]
bool LineBuffer::readline_buf_nmea()
{
    int len;
    while((len=readline_buf()))
        if(check_nmea_cksum(buf[!b], len))
            return true;
    return false;
}

/* return length if a line is in buf[!b] */
int LineBuffer::readline_buf()
{
    while(pos < len) {
        if(buf[b][pos] != '\n') {
            pos++;
            continue;
        }

        int bpos = pos;
        if(pos > 0 && buf[b][pos-1] == '\r')
            bpos--;
        buf[b][bpos] = 0;
        
        len -= pos + 1;
        memcpy(buf[!b], buf[b]+pos+1, len);
        pos = 0;
        b = !b;

        return bpos;
    }
    return 0;
}
