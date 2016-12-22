/* File: ugfx.i */
%module ugfx

%{
#include "ugfx.h"
%}

uint32_t color(int r, int g, int b);

class surface
{
public:
    surface(surface *s);
    surface(int w, int h, int internal_bypp, const char *data32);
    surface(const char* filename);
    virtual ~surface();

    void store_grey(const char *filename);
    void blit(surface *src, int xoff, int yoff);
    void magnify(int factor);
    void putpixel(int x, int y, uint32_t c);
    void line(int x1, int y1, int x2, int y2, uint32_t c);
    void hline(int x1, int x2, int y, uint32_t c);
    void vline(int x, int y1, int y2, uint32_t c);
    void rectangle(int x1, int y1, int x2, int y2, uint32_t c);
    void box(int x1, int y1, int x2, int y2, uint32_t c);
    void invert(int x1, int y1, int x2, int y2);
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
