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

uint32_t gp(surface *src, int x, int y)
{
    if(x >= src->width || y >= src->height)
        return 255;
    return src->getpixel(x, y);
}

// not very efficient but interpolate copy surface of difference sizes
void stretch(surface *dest, surface *src)
{
    for(int y = 0; y<dest->height; y++)
        for(int x=0; x<dest->width; x++) {
            float xi = (float)x*src->width/dest->width;
            float yi = (float)y*src->height/dest->height;
            int xi0 = (int)xi, yi0 = (int)yi;
            float xf = xi-xi0, yf = yi-yi0;
            uint32_t c0 = (gp(src, xi0, yi0)*(1-xf) + gp(src, xi0+1, yi0)*xf);
            uint32_t c1 = (gp(src, xi0, yi0+1)*(1-xf) + gp(src, xi0+1, yi0+1)*xf);
            uint32_t c = c0*(1-yf) + c1*yf;
                
            if(c > 160)
                c = 255;

            dest->putpixel(x, y, c);
        }
}

int main(int argc, char *argv[])
{
    surface *framebuffer;
#ifdef WIRINGPI    
    if(argc > 1) {
        if(!strcmp(argv[1], "spi"))
            framebuffer = new spiscreen(-1);
        else if(!strcmp(argv[1], "nokia5110"))
            framebuffer = new spiscreen(0);
        else if(!strcmp(argv[1], "jlx12864"))
            framebuffer = new spiscreen(1);
        else {
            printf("unknown lcd %s\n", argv[1]);
            exit(1);
        }
    } else
#endif
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

    int fac = facf;

    logo->blit(version, logo->width - version->width, logo->height - version->height);

    surface *logom = new surface(framebuffer);
    //logom->magnify(logo, fac);
    stretch(logom, logo);

    
    logom->invert(0, 0, logom->width, logom->height);

    framebuffer->fill(255);
    framebuffer->blit(logom, 0, 0);
    static_cast<spiscreen*>(framebuffer)->contrast = 60;
    framebuffer->refresh();

#if 0 // for testing
    while(1) {
        usleep(100000);
/*        framebuffer->fill(0);
        framebuffer->refresh();
        usleep(1000000);
        framebuffer->fill(255);
        framebuffer->refresh();
        usleep(1000000);
*/
        framebuffer->blit(logom, 0, 0);
        framebuffer->refresh();
    }
#endif
    delete framebuffer;
}
