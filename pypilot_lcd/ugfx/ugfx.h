/* Copyright (C) 2016 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

#include <linux/fb.h>

uint32_t color(int r, int g, int b);

class surface
{
public:
    surface(surface *s);
    surface(int w, int h, int internal_bypp, const char *data32);
    virtual ~surface();
    void blit(surface *src, int xoff, int yoff);
    void magnify(int factor);
    void putpixel(int x, int y, uint32_t c);
    void line(int x1, int y1, int x2, int y2, uint32_t c);
    void hline(int x1, int x2, int y, uint32_t c);
    void vline(int x, int y1, int y2, uint32_t c);
    void rectangle(int x1, int y1, int x2, int y2, uint32_t c);
    void box(int x1, int y1, int x2, int y2, uint32_t c);
    void fill(uint32_t c);

    int width, height, bypp;
    char *p;
    int xoffset, yoffset, line_length;

protected:
    surface() {}
};


class display : public surface
{
public:
    display(const char *device);
    virtual ~display();

    struct fb_fix_screeninfo finfo;
    struct fb_var_screeninfo vinfo;

    char *fbp = 0;
    int fbfd = 0;
    long int screensize = 0;
};
