/* Copyright (C) 2016 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include "ugfx.h"

static uint16_t color16(uint32_t c)
{
    return (c&0xfc)<<11 | (c&0xfd00) >> 3 | (c&0xfc0000) >> 16;
}

uint32_t color(int r, int g, int b)
{
    return (r<<16) + (g<<8) + (b);
}

surface::surface(surface *s)
{
    xoffset = yoffset = 0;
    width = s->width, height = s->height;
    bypp = s->bypp;
    p = new char [width*height*bypp];
    memcpy(p, s->p, width*height*bypp);
    line_length = width*bypp;
}

surface::surface(int w, int h, int internal_bypp, const char *data32)
{
    xoffset = yoffset = 0;
    width = w, height = h;
    bypp = internal_bypp;
    p = new char [width*height*bypp];
    line_length = width*bypp;

    if (!data32)
        return;

    if (bypp == 2) {
        int i = 0;
        for(int y = 0; y<height; y++)
            for(int x = 0; x<width; x++){
                uint32_t c = *(uint32_t*)(data32 + i);
                uint16_t t = (c&0xfc)<<11 | (c&0xfd00) >> 3 | (c&0xfc0000) >> 16;
                long location = x * bypp + y * line_length;
                *((uint16_t*)(p + location)) = t;
                i++;
            }
    } else
        memcpy(p, data32, 4*width*height);
}

surface::~surface()
{
    delete [] p;
}

void surface::blit(surface *src, int xoff, int yoff)
{
    if(bypp != src->bypp) {
        printf("incompatible surfaces cannot be blit\n");
        return;
    }

    int w = src->width, h = src->height, xsoff = 0, ysoff = 0;

    long src_location = 0;
    if (xoff < 0) {
        src_location -= bypp*xoff;
        w += xoff;
        xoff = 0;
    }

    if (yoff < 0) {
        src_location -= src->line_length*yoff;
        h += yoff;
        yoff = 0;
    }
    
    if (xoff + w > width)
        w = width - xoff;
    if (yoff + h > height)
        h = height - yoff;

    for(int y = 0; y<h; y++) {
        long dest_location = (xoff+xoffset) * bypp + (y+yoff+yoffset) * line_length;
        memcpy(p + dest_location, src->p + src_location, bypp*w);
        src_location += src->line_length;
    }
}

void surface::magnify(int factor)
{
    char *q = new char [factor*factor*width*height*bypp];

    long sl = 0;
    for(int y = 0; y<height; y++)
        for(int x = 0; x<width; x++) {
            long dl = factor * (x*bypp + y*factor*line_length);
            for(int i = 0; i < factor; i++) {
                for(int j = 0; j < factor; j++) {
                    memcpy(q + dl, p + sl, bypp);
                    dl += bypp;
                }
                dl += factor*(line_length - bypp);
            }
            sl+=bypp;
        }

    delete [] p;

    width *= factor, height *= factor, line_length *= factor;
    p = q;
}

void surface::putpixel(int x, int y, uint32_t c)
{
    long dl = x * bypp + y * line_length;
    if(bypp == 2) {
        uint16_t t = color16(t);
        *(uint16_t*)(p + dl) = c;
    } else
        *(uint32_t*)(p + dl) = c;
}

void surface::line(int x1, int y1, int x2, int y2, uint32_t c)
{
    if (abs(x2 - x1) > abs(y2 - y1)) {
        if (x2 < x1)
            line(x2, y2, x1, y1, c);
        else
            for(int x = x1; x < x2; x++) {
                int y = (y2 - y1)*(x - x1)/(x2 - x1) + y1;
                putpixel(x, y, c);
            }
    } else {
        if (y2 < y1)
            line(x2, y2, x1, y1, c);
        else
            for(int y = y1; y < y2; y++) {
                int x = (x2 - x1)*(y - y1)/(y2 - y1) + x1;
                putpixel(x, y, c);
            }
    }
}

void surface::hline(int x1, int x2, int y, uint32_t c)
{
    if(bypp == 2) {
        uint16_t t = color16(c);
        for(int x = x1; x <= x2; x++)
            *(uint16_t*)(p + y*line_length + x*bypp) = t;
    } else
        for(int x = x1; x <= x2; x++)
            *(uint32_t*)(p + y*line_length + x*bypp) = c;
}

void surface::vline(int x, int y1, int y2, uint32_t c)
{
    if(bypp == 2) {
        uint16_t t = color16(t);
        for(int y = y1; y <= y2; y++)
            *(uint16_t*)(p + y*line_length + x*bypp) = t;
    } else
        for(int y = y1; y <= y2; y++)
            *(uint32_t*)(p + y*line_length + x*bypp) = c;
}

void surface::rectangle(int x1, int y1, int x2, int y2, uint32_t c)
{
    hline(x1, x2, y1, c);
    hline(x1, x2, y2, c);
    vline(x1, y1, y2, c);
    vline(x2, y1, y2, c);
}

void surface::box(int x1, int y1, int x2, int y2, uint32_t c)
{
    if(bypp == 2) {
        uint16_t t = color16(t);
        for(int y = y1; y <= y2; y++)
            for(int x = x1; x <= x2; x++)
                *(uint16_t*)(p + y*line_length + x*bypp) = t;
    } else
        for(int y = y1; y <= y2; y++)
            for(int x = x1; x <= x2; x++)
                *(uint32_t*)(p + y*line_length + x*bypp) = c;
}

void surface::fill(uint32_t c)
{
    box(0, 0, width-1, height-1, c);
}

display::display(const char *device)
{
    int x = 0, y = 0;
    
    // Open the file for reading and writing
    fbfd = open(device, O_RDWR);
    if (fbfd == -1) {
        perror("Error: cannot open framebuffer device");
        exit(1);
    }
    printf("The framebuffer device was opened successfully.\n");

    // Get fixed screen information
    if (ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) == -1) {
        perror("Error reading fixed information");
        exit(2);
    }

    // Get variable screen information
    if (ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo) == -1) {
        perror("Error reading variable information");
        exit(3);
    }

    printf("%dx%d, %dbpp\n", vinfo.xres, vinfo.yres, vinfo.bits_per_pixel);

    
    // Figure out the size of the screen in bytes
    screensize = vinfo.xres * vinfo.yres * vinfo.bits_per_pixel/8;

    // Map the device to memory
    fbp = (char *)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
    if ((int)fbp == -1) {
        perror("Error: failed to map framebuffer device to memory");
        exit(4);
    }
    printf("The framebuffer device was mapped to memory successfully.\n");

    p = fbp;
    width = vinfo.xres;
    height = vinfo.yres;
    bypp = vinfo.bits_per_pixel/8;
    xoffset = vinfo.xoffset;
    yoffset = vinfo.yoffset;
    line_length = finfo.line_length;
}

display::~display()
{
    munmap(fbp, screensize);
    p = 0;
    close(fbfd);
}
