/* Copyright (C) 2016 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>

#include "ugfx.h"


#define LOAD() \
    const char *data = header_data; \
    unsigned char pixel[4] = {0}; \
    char data32[4*width*height], *p = data32; \
    for(int i = 0; i<width*height; i++) { \
        HEADER_PIXEL(data, pixel); \
        memcpy(p, pixel, 4); \
        p += 4; \
    } \
    return new surface(width, height, bypp, data32);

surface *load_logo(int bypp)
{    
    #include "pypilot_logo.h"
    LOAD()
    #undef HEADER_PIXEL
}

surface *load_version(int bypp)
{    
    #include "pypilot_version.h"
    LOAD()
    #undef HEADER_PIXEL
}

int main(int argc, char *argv[])
{
    surface *framebuffer;
    if(argc > 1 && !strcmp(argv[1], "nokia5110"))
        framebuffer = new spiscreen();
    else
        framebuffer = new screen("/dev/fb0");

#if 1
    surface *logo = load_logo(framebuffer->bypp);
    surface *version = load_version(framebuffer->bypp);
#else
    const char *path = "/home/tc/.pypilot/splash.surf";
    surface *logo = new surface(path);
    if(logo->bypp != framebuffer->bypp) {
        logo = load_logo(framebuffer->bypp);
        logo->store_grey(path);
        delete logo;

        logo = new surface(path);
        if(logo->bypp != framebuffer->bypp) {
            printf("failed to load %s\n", path);
            exit(1);
        }
    }
#endif    
    double facw = (float)framebuffer->width / logo->width, fach = (float)framebuffer->height / logo->height;
    float facf = facw < fach ? facw : fach;
    surface *logom = new surface(framebuffer);

    int fac = facf;

    logo->blit(version, logo->width - version->width, logo->height - version->height);

    logom->magnify(logo, fac);
    logom->invert(0, 0, logom->width, logom->height);

    framebuffer->fill(255);
    framebuffer->blit(logom, 0, 0);
    framebuffer->refresh();

#if 1
    for(int i=0; i<20; i++) {
        framebuffer->fill(0);
        framebuffer->refresh();
        usleep(100000);
        framebuffer->fill(255);
        framebuffer->refresh();
        usleep(100000);
        framebuffer->blit(logom, 0, 0);
        framebuffer->refresh();
        usleep(100000);
    }
#endif
    delete framebuffer;
}
