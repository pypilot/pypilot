/* Copyright (C) 2017 Sean D'Epagnier <seandepagnier@gmail.com>
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

static uint16_t color16gray(uint8_t c)
{
    return (c&0xfc)<<11 | (c&0xfc)<<5 | (c&0xfc);
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

    if(bypp == 1) {
        int i = 0;
        for(int y = 0; y<height; y++)
            for(int x = 0; x<width; x++){
                uint32_t c = ((uint32_t*)data32)[i];
                long location = x + y * line_length;
                *((uint8_t*)(p + location)) = c&0xff;
                i++;
            }
    } else
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
    } else if(bypp == 4)
        memcpy(p, data32, 4*width*height);
    else
        fprintf(stderr, "bypp incompatible with input data\n");
}

uint32_t cksum(const char *gray_data, int size)
{
    int32_t crc = 0xab325f12, mask = 0;  // random start
    for(int i=0; i<size; i++) {
        crc ^= gray_data[i];
        for(int b=0; b<7; b++) {
            mask = -(crc & 1);
            crc = (crc >> 1) ^ (0xEDB88320 & mask);
        }
    }
    return crc;
}

surface::surface(const char* filename)
{
    width = height = bypp = 0;
    p = NULL;

    FILE *f = fopen(filename, "r");
    if(!f)
        return;

    uint16_t file_bypp, colors;
    if(fread(&width, 2, 1, f) != 1 || fread(&height, 2, 1, f) != 1 ||
       fread(&file_bypp, 2, 1, f) != 1 || fread(&colors, 2, 1, f) != 1) {
        fprintf(stderr, "failed reading surface header\n");
        goto fail;
    }

    if(width*height > 65536) {
        fprintf(stderr, "invalid surface size\n");
        goto fail;
    }
    
    xoffset = yoffset = 0;
    p = new char [width*height*file_bypp];
    line_length = width*file_bypp;
    bypp = file_bypp;

    if(colors != 1) // only greyscale supported
        goto fail;  

    {
        char gray_data[width*height];
        unsigned int i=0;
        while(i<sizeof gray_data) {
            uint8_t run, value;
            if(fread(&run, 1, 1, f) != 1 || fread(&value, 1, 1, f) != 1) {
                fprintf(stderr, "failed reading surface data\n");
                goto fail;
            }
            while(run-- > 0)
                gray_data[i++] = value;
        }

        uint32_t computed_crc = cksum(gray_data, sizeof gray_data);
        uint32_t crc = 0;
        if(fread(&crc, 4, 1, f) != 1 || computed_crc != crc) {
            printf("crc doesn't match %x %x\n", computed_crc, crc);
            goto fail;
        }

        if(file_bypp == 1)
            memcpy(p, gray_data, width*height);
        else if(file_bypp == 2)
            for(int i = 0; i<width*height; i++)
                ((uint16_t*)p)[i] = color16gray(gray_data[i]);
        else if(file_bypp == 4)
            for(int i = 0; i<width*height; i++)
                memset(p + 4*i, gray_data[i], 3);
        else
            fprintf(stderr, "bypp incompatible reading %s\n", filename);
    }

    return;

fail:
    fprintf(stderr, "failed ot open %s\n", filename);
    fclose(f);
    bypp = 0;
}

surface::~surface()
{
    delete [] p;
}

void surface::store_grey(const char *filename)
{
    char gray_data[width*height];
    for(unsigned int i=0; i<sizeof gray_data; i++)
        if(bypp == 1)
            gray_data[i] = p[i];
        else if(bypp == 2)
            gray_data[i] = p[2*i]&0xfc;
        else if(bypp == 4)
            gray_data[i] = p[4*i];
        else
            fprintf(stderr, "bypp incompatible storing %s\n", filename);

    FILE *f = fopen(filename, "w");
    uint16_t colors = 1; // grey
    fwrite(&width, 1, 2, f);
    fwrite(&height, 1, 2, f);
    fwrite(&bypp, 1, 2, f);
    fwrite(&colors, 1, 2, f);

    char last = 0;
    uint8_t run = 0;
    for(unsigned int i=0; i<sizeof gray_data; i++) {
        if(gray_data[i] == last) {
            if(run == 255) {
                fwrite(&run, 1, 1, f);
                fwrite(&last, 1, 1, f);
                run = 0;
            }
            run++;
        } else {
            if(run > 0) {
                fwrite(&run, 1, 1, f);
                fwrite(&last, 1, 1, f);
            }
            last = gray_data[i];
            run = 1;
        }
    }
    
    fwrite(&run, 1, 1, f);
    fwrite(&last, 1, 1, f);
    uint32_t crc = cksum(gray_data, sizeof gray_data);
    fwrite(&crc, 1, 4, f);
    fclose(f);
}

void surface::blit(surface *src, int xoff, int yoff, bool flip)
{
    if(bypp != src->bypp) {
        printf("incompatible surfaces cannot be blit\n");
        return;
    }

    int w = src->width, h = src->height;

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

    if(w <= 0 || h <= 0)
        return;

    if(flip)
        for(int y = h-1; y>=0; y--) {
            long dest_location = (xoff+xoffset) * bypp + (y+yoff+yoffset) * line_length;
            for(int x=0; x<w; x++)
                memcpy(p + dest_location + x*bypp, src->p + src_location + bypp*(w-x-1), bypp);
            src_location += src->line_length;
        }
    else
        for(int y = 0; y<h; y++) {
            long dest_location = (xoff+xoffset) * bypp + (y+yoff+yoffset) * line_length;
            memcpy(p + dest_location, src->p + src_location, bypp*w);
            src_location += src->line_length;
        }
}

void surface::magnify(surface *src, int factor)
{
    if(factor == 1) {
        blit(src, 0, 0);
        return;
    }

    if(width < src->width*factor ||
       height < src->height*factor) {
        printf("magnify surface not large enough\n");
        return;
    }

#define MAG(BYPP) \
    long sl = 0; \
    for(int y = 0; y<src->height; y++) \
        for(int x = 0; x<src->width; x++) { \
            long dl = factor * (x*BYPP + y*line_length); \
            for(int i = 0; i < factor; i++) { \
                for(int j = 0; j < factor; j++) { \
                    memcpy(p + dl, src->p + sl, BYPP); \
                    dl += BYPP; \
                } \
                dl += line_length - factor*BYPP; \
            } \
            sl+=BYPP; \
        }

    // compiler generates much better code if bypp is constant
    if(bypp == 2) {
        MAG(2);
    } else if(bypp == 4) {
        MAG(4);
    } else
        fprintf(stderr, "bypp incompatible with magnify\n");
}

void surface::putpixel(int x, int y, uint32_t c)
{
    long dl = x * bypp + y * line_length;
    switch(bypp) {
    case 1: *(uint8_t*)(p + dl) = c&0xff;      break;
    case 2: *(uint16_t*)(p + dl) = color16(c); break;
    case 4: *(uint32_t*)(p + dl) = c;          break;
    default:
        fprintf(stderr, "bypp incompatible with putpixel\n");
    }
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
    switch(bypp) {
    case 1:
        memset(p + y*line_length, c&0xff, x2-x1);
        break;
    case 2:
    {
        uint16_t t = color16(c);
        for(int x = x1; x <= x2; x++)
            *(uint16_t*)(p + y*line_length + x*bypp) = t;
    } break;
    case 4:
        for(int x = x1; x <= x2; x++)
            *(uint32_t*)(p + y*line_length + x*bypp) = c;
        break;
    default:
        fprintf(stderr, "bypp incompatible with hline\n");
    }
}

void surface::vline(int x, int y1, int y2, uint32_t c)
{
    switch(bypp) {
    case 1:
    {
        uint8_t t = c&0xff;
        for(int y = y1; y <= y2; y++)
            *(uint8_t*)(p + y*line_length + x) = t;
    } break;
    case 2:
    {
        uint16_t t = color16(c);
        for(int y = y1; y <= y2; y++)
            *(uint16_t*)(p + y*line_length + x*bypp) = t;
    } break;
    case 4:
        for(int y = y1; y <= y2; y++)
            *(uint32_t*)(p + y*line_length + x*bypp) = c;
        break;
    default:
        fprintf(stderr, "bypp incompatible with vline\n");
    }
}

void surface::box(int x1, int y1, int x2, int y2, uint32_t c)
{
    switch(bypp) {
    case 1:
    {
        uint16_t t = c&0xff;
        for(int y = y1; y <= y2; y++)
            memset(p + y*line_length + x1, t, x2-x1+1);
    } break;
    case 2:
    {
        uint16_t t = color16(c);
        for(int y = y1; y <= y2; y++)
            for(int x = x1; x <= x2; x++)
                *(uint16_t*)(p + y*line_length + x*bypp) = t;
    } break;
    case 4:
        for(int y = y1; y <= y2; y++)
            for(int x = x1; x <= x2; x++)
                *(uint32_t*)(p + y*line_length + x*bypp) = c;
        break;
    default:
        fprintf(stderr, "bypp incompatible with box\n");
    }
}

void surface::invert(int x1, int y1, int x2, int y2)
{
    x1 = x1 > 0 ? x1 : 0;
    x2 = x2 < width ? x2 : width-1;
    y1 = y1 > 0 ? y1 : 0;
    y2 = y2 < height ? y2 : height-1;

    int len = (x2-x1+1)*bypp;
    
    for(int y = y1; y <= y2; y++) {
        int i1 = y*line_length + x1*bypp;
        int i2 = i1 + len;
        int i = i1;
#if 1 // optimization
        /* align to 8 byte boundary */
        while(i&7 && i<i2) {
            p[i] = ~p[i];
            i++;
        }

        /* invert 8 bytes at a time */
        while(i+7 < i2) {
            *(uint64_t*)(p+i) = ~*(uint64_t*)(p+i);
            i += 8;
        }
#endif
        /* remainder bytes */
        while(i<i2) {
            p[i] = ~p[i];
            i++;
        }
    }
}

void surface::fill(uint32_t c)
{
    box(0, 0, width-1, height-1, c);
}

void surface::binary_write(int fileno)
{
    if(bypp != 1)
        return;
  
    char binary[width*height/8];
    for(int col = 0; col<6; col++)
        for(int y = 0; y < height; y++) {
            int index = y + (5-col)*height;
            uint8_t bits = 0;
            for(int bit = 0; bit<8; bit++) {
                bits <<= 1;
                if(*(uint8_t*)(p + y*line_length + col*8+bit))
                    bits |= 1;
            }
            binary[index] = bits;
        }
#if 1
    for (unsigned int pos=0; pos<sizeof binary; pos += 64) {
        int len = 64;
        if (sizeof binary - pos < 64)
            len = sizeof binary - pos;
        write(fileno, binary + pos, len);
    }   
#else
    write(fileno, binary, sizeof binary);
#endif
}

#ifdef __ARMEL__
#include <wiringPi.h>
void surface::binary_write_sw(int sclk, int mosi)
{
    static int setup;
    if(!setup) {
        wiringPiSetupPhys();
        setup = 1;
    }
    for(int col=0; col<6; col++)
        for(int y=0; y<84; y++)
            for(int bit=0; bit<8; bit++) {
                // Write bit to MOSI.
                int x = (5-col)*8+bit;
                digitalWrite(mosi, !!p[y*line_length + x*bypp]);

                digitalWrite(sclk, HIGH);
                digitalWrite(sclk, LOW);
            }

}
#else
void surface::binary_write_sw(int sclk, int mosi)
{
    fprintf(stderr, "no wiring pi\n");
}
#endif

int surface::getpixel(int x, int y)
{
    if(bypp == 1)
        return *(uint8_t*)(p + y*line_length + x*bypp);
    else if(bypp == 2)
        return *(uint16_t*)(p + y*line_length + x*bypp);
    else if(bypp == 4)
        return *(uint32_t*)(p + y*line_length + x*bypp);

    fprintf(stderr, "bypp incompatible with getpixel\n");
    exit(1);
}


screen::screen(const char *device)
{
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
    screensize = vinfo.yres * finfo.line_length;

    // Map the device to memory
    fbp = (char*)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
    if (fbp == (void*)-1) {
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

screen::~screen()
{
    munmap(fbp, screensize);
    p = 0;
    close(fbfd);
}


#ifdef __ARMEL__
#include <wiringPiSPI.h>

#define SPI_DEVICE 0
#define DC 6 //25
#define RST 5 //24

#define LCDWIDTH 84
#define LCDHEIGHT 48
#define ROWPIXELS LCDHEIGHT/6
#define PCD8544_POWERDOWN 0x04
#define PCD8544_ENTRYMODE 0x02
#define PCD8544_EXTENDEDINSTRUCTION 0x01
#define PCD8544_DISPLAYBLANK 0x0
#define PCD8544_DISPLAYNORMAL 0x4
#define PCD8544_DISPLAYALLON 0x1
#define PCD8544_DISPLAYINVERTED 0x5
#define PCD8544_FUNCTIONSET 0x20
#define PCD8544_DISPLAYCONTROL 0x08
#define PCD8544_SETYADDR 0x40
#define PCD8544_SETXADDR 0x80
#define PCD8544_SETTEMP 0x04
#define PCD8544_SETBIAS 0x10
#define PCD8544_SETVOP 0x80

class PCD8544
{
public:
    PCD8544() {
	setenv("WIRINGPI_CODES", "1", 1);
        if(wiringPiSetup () < 0) {
            printf("wiringPiSetup Failed (no permissions?)\n");
            exit(1);
        }

        pinMode(RST, OUTPUT);
        pinMode(DC, OUTPUT);

	for(int port=0; port<2; port++)
	    if((spifd = wiringPiSPISetup(port, 4000000)) != -1)
		break;
	  
	if(spifd == -1) {
            fprintf(stderr, "failed to open spi device");
            exit(1);
	}
    }

    ~PCD8544() {
        close(spifd);
    }

    void command(uint8_t c) {
        digitalWrite (DC, LOW) ;	// Off
        write(spifd, &c, 1);
    }

    void extended_command(uint8_t c) {
        command(PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION);
        command(c);

        command(PCD8544_FUNCTIONSET);
        command(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);
    }
    
    void begin(int contrast, int bias=4) {
        reset();
        set_bias(bias);
        set_contrast(contrast);
    }

    void reset() {
        digitalWrite (RST, LOW);
        usleep(100000);
        digitalWrite (RST, HIGH);
    }
    
    void set_contrast(int contrast) {
        contrast = contrast > 0x7f ? 0x7f : contrast;
        contrast = contrast < 0 ? 0 : contrast;
        extended_command(PCD8544_SETVOP | contrast);
    }

    void set_bias(int bias) {
        extended_command(PCD8544_SETBIAS | bias);
    }

    int spifd;
};

nokia5110screen::nokia5110screen()
    : surface(48, 84, 1, NULL)
{
    disp = new PCD8544();
    contrast = 50;
    disp->begin(contrast);
}

nokia5110screen::~nokia5110screen()
{
    delete disp;
}


void nokia5110screen::refresh()
{
    disp->set_bias(4);
    disp->set_contrast(contrast);

    disp->command(PCD8544_SETYADDR);
    disp->command(PCD8544_SETXADDR);

    digitalWrite (DC, HIGH) ;
    binary_write(disp->spifd);
}

#endif
