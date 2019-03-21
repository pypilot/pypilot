/* File: ugfx.i */
%module ugfx

%{
#include "ugfx.h"
%}

unsigned int color(int r, int g, int b);

class surface
{
public:
    surface(surface *s);
    surface(int w, int h, int internal_bypp, const char *data32);
    surface(const char* filename);
    virtual ~surface();

    void store_grey(const char *filename);
    void blit(surface *src, int xoff, int yoff, bool flip=false);
    void magnify(surface *src, int factor);
    void putpixel(int x, int y, unsigned int c);
    void line(int x1, int y1, int x2, int y2, unsigned int c);
    void hline(int x1, int x2, int y, unsigned int c);
    void vline(int x, int y1, int y2, unsigned int c);
    void box(int x1, int y1, int x2, int y2, unsigned int c);
    void invert(int x1, int y1, int x2, int y2);
    void fill(unsigned int c);
    virtual void refresh() {}
    void binary_write(int fileno);
    void binary_write_sw(int sclk, int mosi);

    int width, height, bypp;
    char *p;
    int getpixel(int x, int y);
    unsigned char *ptr() { return (unsigned char*)p; }
 
    int xoffset, yoffset, line_length;

protected:
    surface() {}
};


class screen : public surface
{
public:
    screen(const char *device);
    virtual ~screen();

    struct fb_fix_screeninfo finfo;
    struct fb_var_screeninfo vinfo;

    char *fbp = 0;
    int fbfd = 0;
    long int screensize = 0;
};

#ifdef WIRINGPI
class nokia5110screen : public surface
{
public:
    nokia5110screen();
    virtual ~nokia5110screen();

    void refresh();
    int contrast;
};
#endif
