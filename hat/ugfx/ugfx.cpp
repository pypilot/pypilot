/* Copyright (C) 2019 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
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
        ;//        fprintf(stderr, "bypp incompatible with input data\n");
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

surface::surface(const char* filename, int tbypp)
{
    width = height = bypp = 0;
    p = NULL;

    FILE *f = fopen(filename, "r");
    if(!f)
        return;

    uint16_t width16, height16, bypp16, colors16;
    if(fread(&width16, 2, 1, f) != 1 || fread(&height16, 2, 1, f) != 1 ||
       fread(&bypp16, 2, 1, f) != 1 || fread(&colors16, 2, 1, f) != 1) {
        fprintf(stderr, "failed reading surface header\n");
        goto fail;
    }

    width = width16;
    height = height16;
    bypp = tbypp;
    if(width*height > 65536) {
        fprintf(stderr, "invalid surface size\n");
        goto fail;
    }
    
    xoffset = yoffset = 0;
    p = new char [width*height*bypp];
    line_length = width*bypp;

    if(colors16 != 1) // only greyscale supported
        goto fail;  

    {
        char gray_data[width*height];
        unsigned int i=0;
        while(i<sizeof gray_data) {
            uint8_t run, value;
            if(fread(&run, 1, 1, f) != 1 || fread(&value, 1, 1, f) != 1) {
//                fprintf(stderr, "failed reading surface data\n");
                goto fail;
            }
            while(run-- > 0)
                gray_data[i++] = value;
        }

        uint32_t computed_crc = cksum(gray_data, sizeof gray_data);
        uint32_t crc = 0;
        if(fread(&crc, 4, 1, f) != 1 || computed_crc != crc) {
//            printf("crc doesn't match %x %x\n", computed_crc, crc);
            goto fail;
        }

        if(bypp == 1)
            memcpy(p, gray_data, width*height);
        else if(bypp == 2)
            for(int i = 0; i<width*height; i++)
                ((uint16_t*)p)[i] = color16gray(gray_data[i]);
        else if(bypp == 4)
            for(int i = 0; i<width*height; i++)
                memset(p + 4*i, gray_data[i], 3);
        else
            ;//            fprintf(stderr, "bypp incompatible reading %s\n", filename);
    }

    fclose(f);
    return;

fail:
//    fprintf(stderr, "failed ot open %s\n", filename);
    delete [] p;
    fclose(f);
    bypp = 0;
}

surface::~surface()
{
    delete [] p;
}

void surface::store_grey(const char *filename)
{
    FILE *f = fopen(filename, "w");
    if(!f) {
        //        fprintf(stderr, "failed to open for writing %s\n", filename);
        return;
    }

    unsigned int datasize = width*height;
    char grey_data[datasize];
    for(unsigned int i=0; i<datasize; i++)
        if(bypp == 1)
            grey_data[i] = p[i];
        else if(bypp == 2)
            grey_data[i] = p[2*i]&0xfc;
        else if(bypp == 4)
            grey_data[i] = p[4*i];
        else
            ;//            fprintf(stderr, "bypp incompatible storing %s\n", filename);

    uint16_t width16 = width, height16 = height, bypp16 = bypp, colors16 = 1; // grey
    fwrite(&width16, 2, 1, f);
    fwrite(&height16, 2, 1, f);
    fwrite(&bypp16, 2, 1, f);
    fwrite(&colors16, 2, 1, f);

    char last = 0;
    uint8_t run = 0;
    for(unsigned int i=0; i<datasize; i++) {
        if(grey_data[i] == last) {
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
            last = grey_data[i];
            run = 1;
        }
    }
    
    fwrite(&run, 1, 1, f);
    fwrite(&last, 1, 1, f);
    uint32_t crc = cksum(grey_data, datasize);
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
        //        printf("magnify surface not large enough\n");
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
        ;//        fprintf(stderr, "bypp incompatible with magnify\n");
}

void surface::putpixel(int x, int y, uint32_t c)
{
    long dl = x * bypp + y * line_length;
    switch(bypp) {
    case 1: *(uint8_t*)(p + dl) = c&0xff;      break;
    case 2: *(uint16_t*)(p + dl) = color16(c); break;
    case 4: *(uint32_t*)(p + dl) = c;          break;
    default:
        ;//        fprintf(stderr, "bypp incompatible with putpixel\n");
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
        ;//        fprintf(stderr, "bypp incompatible with hline\n");
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
        ;//        fprintf(stderr, "bypp incompatible with vline\n");
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
        ;//        fprintf(stderr, "bypp incompatible with box\n");
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

#ifdef WIRINGPI
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
//    fprintf(stderr, "no wiring pi\n");
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

//    fprintf(stderr, "bypp incompatible with getpixel\n");
    exit(1);
}

#ifdef __linux__
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

screen::screen(const char *device)
{
    // Open the file for reading and writing
    fbfd = open(device, O_RDWR);
    if (fbfd == -1) {
//        perror("Error: cannot open framebuffer device");
        exit(1);
    }
//    printf("The framebuffer device was opened successfully.\n");

    // Get fixed screen information
    if (ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) == -1) {
//        perror("Error reading fixed information");
        exit(2);
    }

    // Get variable screen information
    if (ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo) == -1) {
//        perror("Error reading variable information");
        exit(3);
    }

//    printf("%dx%d, %dbpp\n", vinfo.xres, vinfo.yres, vinfo.bits_per_pixel);

    // Figure out the size of the screen in bytes
    screensize = vinfo.yres * finfo.line_length;

    // Map the device to memory
    fbp = (char*)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
    if (fbp == (void*)-1) {
//        perror("Error: failed to map framebuffer device to memory");
        exit(4);
    }
//    printf("The framebuffer device was mapped to memory successfully.\n");

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
#endif

#ifdef WIRINGPI
#include <wiringPiSPI.h>

class spilcd
{
public:
    spilcd(int _rst, int _dc)
        : rst(_rst), dc(_dc) {
    	setenv("WIRINGPI_CODES", "1", 1);
        if(wiringPiSetup () < 0) {
//            printf("wiringPiSetup Failed (no permissions?)\n");
            exit(1);
        }

        pinMode(rst, OUTPUT);
        pinMode(dc, OUTPUT);

	for(int port=0; port<2; port++)
	    if((spifd = wiringPiSPISetup(port, 100000)) != -1)
		break;
	  
	if(spifd == -1) {
//            fprintf(stderr, "failed to open spi device");
            exit(1);
	}
    }

    virtual ~spilcd() {
        close(spifd);
    }


    void command(uint8_t c) {
        digitalWrite (dc, LOW) ;	// Off
        write(spifd, &c, 1);
    }

    void reset() {
        digitalWrite (rst, LOW);
        usleep(200000);
        digitalWrite (rst, HIGH);
    }

    virtual void refresh(int contrast, surface *s) = 0;

    int spifd, rst, dc;
};

#else
class spilcd
{
public:
    spilcd(int _rst, int _dc)
        : rst(_rst), dc(_dc) {
    }

    virtual ~spilcd() {
    }
    void command(uint8_t c) {
    }

    void reset() {
    }

    virtual void refresh(int contrast, surface *s) = 0;

    int spifd, rst, dc;
};
#endif

#ifdef WIRINGPI
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

class PCD8544 : public spilcd
{
public:
    PCD8544() : spilcd(RST, DC) {}
    virtual ~PCD8544() {} 

    void extended_command(uint8_t c) {
        command(PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION);
        command(c);

        command(PCD8544_FUNCTIONSET);
        command(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);
    }
    
    void set_contrast(int contrast) {
        contrast = contrast > 0x7f ? 0x7f : contrast;
        contrast = contrast < 0 ? 0 : contrast;
        extended_command(PCD8544_SETVOP | contrast);
    }

    void set_bias(int bias) {
        extended_command(PCD8544_SETBIAS | bias);
    }

    void refresh(int contrast, surface *s) {
        if(s->bypp != 1)
            return;

        set_bias(4);
        set_contrast(contrast);

        command(PCD8544_SETYADDR);
        command(PCD8544_SETXADDR);

        digitalWrite (dc, HIGH) ;

        int size = 84*48/8;

        char binary[size];
        for(int col = 0; col<6; col++)
            for(int y = 0; y < s->height; y++) {
                int index = y + (5-col)*s->height;
                uint8_t bits = 0;
                for(int bit = 0; bit<8; bit++) {
                    bits <<= 1;
                    if(*(uint8_t*)(s->p + y*s->line_length + col*8+bit))
                        bits |= 1;
                }
                binary[index] = bits;
            }
        
        // write up to 64 bytes at a time
        for (int pos=0; pos<size; pos += 64) {
            int len = 64;
            if (size - pos < 64)
                len = size - pos;
            write(spifd, binary + pos, len);
        }
    }
};

// JLX12864G-086


#define LCD_C LOW
#define LCD_D HIGH

#define LCD_X 128
#define LCD_Y 64
#define LCD_CMD 0

const int rstPIN  = 5;    // RST
const int  rsPIN  = 6;    // RS

class JLX12864G : public spilcd
{
public:
    JLX12864G() : spilcd(rstPIN, rsPIN) {}
    virtual ~JLX12864G() {}
    void refresh(int contrast, surface *s) {
//        command(0x25); // Coarse Contrast, setting range is from 20 to 27
//      command(0x81); // Trim Contrast
//        command(0x1f); // Trim Contrast value range can be set from 0 to 63

        unsigned char cmd[] = {0xe2, // Soft Reset
                            0x2c, // Boost 1
                            0x2e, // Boost 2
                            0x2f, // Boost 3
                            0xa2, // 1/9 bias ratio
                            0xc0, // Line scan sequence : from top to bottom
                            0xa0, // column scan order : from left to right
                            0xa6, // not reverse
                            0xa4, // not all on
                            0x40, // start of first line
                            0xaf}; // Open the display

        digitalWrite (dc, LOW) ;	// Off
        write(spifd, cmd, sizeof cmd);
        digitalWrite (dc, HIGH) ;	// Off

        unsigned char binary[128*64];//width*height/8];
        for(int col = 0; col<8; col++)
            for(int y = 0; y < 128; y++) {
                int index = y + col*s->height;
                uint8_t bits = 0;
                for(int bit = 0; bit<8; bit++) {
                    bits <<= 1;
//                    if(*(uint8_t*)(s->p + y*s->line_length + col*8+7-bit))
                    if(*(uint8_t*)(s->p + (127-y)*s->line_length + (7-col)*8+bit))
                        bits |= 1;
                }
                binary[index] = bits;
            }

//        for(int k=0; k<10; k++)
        for(uint8_t i=0;i<8;i++)
        {
            unsigned char c1 = 0xb0+i;
            unsigned char cmd[] = {c1, 0x10, 0x00};
            digitalWrite (dc, LOW) ;	// Off
            write(spifd, cmd, sizeof cmd);
            digitalWrite (dc, HIGH) ;	// Off
#if 0
            unsigned char *address = binary + i*128; //pointer
            for (unsigned int pos=0; pos<128; pos ++) {
                char data[1] = {binary[i*128+pos]};
                write(spifd, data, 1);
                address++;
            }
#else
            unsigned char *address = binary + i*128; //pointer
            write(spifd, address, 128);
#endif
        }
     }
};

#endif

static int detect(int driver) {
    if(driver != -1)
        return driver;
    
    FILE *f = fopen("/home/tc/.pypilot/lcddriver", "r");
    if(!f)
        return 0;
    char drv[64];
    fgets(drv, sizeof drv, f);
    if(!strcmp(drv, "jlx12864"))
        driver = 1;
    else
        driver = 0;
    fclose(f);
    
    return driver;
}
const int spilcdsizes[][2] = {{48, 84}, {64, 128}};

spiscreen::spiscreen(int driver)
    : surface(spilcdsizes[detect(driver)][0], spilcdsizes[detect(driver)][1], 1, NULL)
{
    driver = detect(driver);
    switch (driver) {
#ifdef WIRINGPI
    case 0: disp = new PCD8544(); break;
    case 1: disp = new JLX12864G(); break;
#endif
    default:
        //  fprintf(stderr, "invalid driver: %d", driver);
        exit(1);
    }
    contrast = 60;
    disp->reset();
}

spiscreen::~spiscreen()
{
    delete disp;
}

void spiscreen::refresh()
{
    disp->refresh(contrast, this);
}

