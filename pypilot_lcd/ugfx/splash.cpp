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

surface *load_logo(int bypp)
{    
    #include "pypilot_logo.h"
    char *data = header_data;
    char pixel[4] = {0};
    char data32[4*width*height], *p = data32;

    for(int i = 0; i<width*height; i++) {
        HEADER_PIXEL(data, pixel);
        memcpy(p, pixel, 4);
        p += 4;
    }
    
    return new surface(width, height, bypp, data32);
}


int main()
{
    display screen("/dev/fb0");

#if 1
    surface *logo = load_logo(screen.bypp);
#else
    const char *path = "/home/tc/.pypilot/splash.surf";
    surface *logo = new surface(path);
    if(logo->bypp != screen.bypp) {
        logo = load_logo(screen.bypp);
        logo->store_grey(path);
        delete logo;

        logo = new surface(path);
        if(logo->bypp != screen.bypp) {
            printf("failed to load %s\n", path);
            exit(1);
        }
    }
#endif    
    int facw = screen.width / logo->width, fach = screen.height / logo->height;
    int fac = facw < fach ? facw : fach;
    logo->magnify(fac);

    for(;;) {
        screen.blit(logo, 0, 0);
        usleep(100000);
    }
}
