/* Copyright (C) 2021 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

#include "JLX12864.h"

#include <Arduino.h>

#include <SPI.h>

#include "charset.h"
#include <avr/pgmspace.h>


#define CMD  LOW
#define DATA HIGH

#define HWSPI // hardware spi is 12x faster


// with only 2k ram, we cannot have a full framebuffer, so split
// into two pages for top and bottom half of display each 64x64
static unsigned char framebuffer[8*64];

JLX12864::JLX12864(unsigned char sclk, unsigned char sdin,
                 unsigned char dc, unsigned char reset,
                 unsigned char sce):
    flip(false),
    pin_sclk(sclk),
    pin_sdin(sdin),
    pin_dc(dc),
    pin_reset(reset),
    pin_sce(sce)
{}


void JLX12864::begin()
{
    width = width;
    height = height;

    // All pins are outputs (these displays cannot be read)...
    pinMode(pin_sclk, OUTPUT);
    pinMode(pin_sdin, OUTPUT);
    pinMode(pin_dc, OUTPUT);
    if(pin_sce != 99)
        pinMode(pin_sce, OUTPUT);

#ifdef HWSPI
#if defined(__AVR_ATmega32__)
    DDRB|=(1<<4)|(1<<5)|(1<<7);
    SPCR=(1<<SPE)|(1<<MSTR)|(1<<SPR0);
#else
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16);
#endif
#endif
    // Reset the controller state...
    digitalWrite(pin_sce, HIGH);

    pinMode(pin_reset, INPUT_PULLUP);
    delay(50);
    pinMode(pin_reset, OUTPUT);
    digitalWrite(pin_reset, LOW);
    delay(50);
    pinMode(pin_reset, INPUT_PULLUP);

    // Clear RAM contents...
    clear();

    delay(50);
    refresh(0);
    refresh(1);
}


void JLX12864::putpixel(uint8_t x, uint8_t y, uint8_t color)
{
    if(x >= 64 || y >= 64)
        return;

    if(flip) {
        x = 63 - x;
        y = 63 - y;
    }

    int col = x/8, bit = 1<<(x%8);
    int ind = col*64 + y;
    if(color)
        framebuffer[ind] |= bit;
    else
        framebuffer[ind] &= ~bit;
}

void JLX12864::rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color)
{
    if(x1 > 63)
        x1 = 63;
    if(x2 > 63)
        x2 = 63;
    if(y1 > 63)
        y1 = 63;
    if(y2 > 63)
        y2 = 63;

    for(uint8_t y=y1; y<=y2; y++)
        for(uint8_t x=x1; x<=x2; x++)
            putpixel(x, y, color);
}

void JLX12864::circle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color)
{
    int f = 1 - r;
    int dx = 0;
    int dy = -r;
    int x = 0;
    int y = r;

    /* draw for edges */
    putpixel(x0, y0 + r, color);
    putpixel(x0, y0 - r, color);
    putpixel(x0 + r, y0, color);
    putpixel(x0 - r, y0, color);

    while(x < y)
    {
        if(f >= 0)
        {
            y--;
            dy++;
            f += dy;
        }
        x++;
        dx++;
        f += dx;

        /* draw all 8 octants */
        putpixel(x0 + x, y0 + y, color);
        putpixel(x0 - x, y0 + y, color);
        putpixel(x0 + x, y0 - y, color);
        putpixel(x0 - x, y0 - y, color);
        putpixel(x0 + y, y0 + x, color);
        putpixel(x0 - y, y0 + x, color);
        putpixel(x0 + y, y0 - x, color);
        putpixel(x0 - y, y0 - x, color);
    }
}

#define SWAP(X, Y) (X ^= Y, Y ^= X, X ^= Y)
#define DRAW_LINE(X, Y) \
   do { \
       if(Y##2 < Y##1) { \
          SWAP(y1, y2); \
          SWAP(x1, x2); \
       } \
       int dp = 2*d##X-d##Y, i; \
       int x = x1, y = y1; \
       int s = X##1 > X##2 ? -1 : 1; \
       for(i = 0; i <= d##Y; i++) { \
           putpixel(x, y, color);         \
          if(dp > 0) { \
             X += s; \
             dp -= 2*d##Y; \
          } \
          Y++; \
          dp += 2*d##X; \
       } \
   } while(0)

void JLX12864::line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color)
{
    int dx = abs(x2-x1), dy = abs(y2-y1);

    if(dy > dx)
       DRAW_LINE(x, y);
    else
       DRAW_LINE(y, x);
}


void JLX12864::SetParameters()
{
    digitalWrite(pin_dc, CMD);

    const int contrast = 90;
    unsigned char cmd[] = {
        0xe2, // Soft Reset
        0xc2, // Line scan sequence : from top to bottom
        0x2c, // Boost 1
        0x2e, // Boost 2
        0x2f, // Boost 3
        0xa2, // 1/9 bias ratio
        
        0x23, // Coarse Contrast, setting range is from 20 to 27
        0x81, // Trim Contrast
        (uint8_t)45, // Trim Contrast value range can be set from 0 to 63                               
        0xa0, // column scan order : from left to right
        0xa6, // not reverse
        0xa4, // not all on
        0x40, // start of first line
//        0xb0,
//        0x10,
        0xaf // Open the display
    };

    for(uint8_t i=0; i<(sizeof cmd) / (sizeof *cmd); i++) {
        SPDR = cmd[i];
        while (!(SPSR & _BV(SPIF)));
    }
        ;//SPI.transfer(cmd[i]);
}

size_t JLX12864::write(uint8_t chr)
{
    // ASCII 7-bit only...
    if (chr >= 0x80)
        return 0;

    struct font f;
    int cfont = curfont;
    memcpy_P(&f, fonts + cfont, sizeof f);
    struct font_character c;

    for(int i=0; i<f.n; i++) {
        memcpy_P(&c, f.characters + i, sizeof c);
        if(c.c == chr)
            goto found;
    }
    return 0; // not found
found:
    {
        unsigned char glyph[c.len];
        memcpy_P(glyph, c.data, c.len);

        uint8_t glyphp = 0;
        uint8_t glyphc = glyph[0] & 0x7F;
        for(int y=0; y<c.h; y++) {
            int yp = ypos + y;
            for(int x=0; x<c.w; x++) {
                uint8_t v = glyph[glyphp]&0x80;
                if(--glyphc == 0) {
                    if(++glyphp == c.len)
                        goto done;
                    glyphc = glyph[glyphp] & 0x7F;
                }
                if(yp < 0 || yp >= 64)
                    continue;
                if(!v)
                    continue;
                int xp = xpos + x;
                if(xp < 0 || xp >= 64)
                    continue;
                putpixel(xp, yp, 255);
            }
        }
    done:
        xpos += c.w + 1;
    }
    return 1;
}

void JLX12864::clear()
{
    memset(framebuffer, 0, sizeof framebuffer);
}

void JLX12864::refresh(uint8_t page)
{
    SetParameters();

    if(page == flip)
        page = 0x10;
    else
        page = 0x14;

    unsigned char *fb = framebuffer;
    
    for(uint8_t c=0;c<8;c++)
    {
        digitalWrite(pin_dc, CMD);
        //SPI.transfer(0xb0 + c);
        SPDR = 0xb0+c;
        while (!(SPSR & _BV(SPIF)));
        //SPI.transfer(page);
        SPDR = page;
        while (!(SPSR & _BV(SPIF)));

        digitalWrite(pin_dc, DATA);
        for(unsigned int i=0; i<64; i++) {
            SPDR = *fb++;
            while (!(SPSR & _BV(SPIF)));
        }
    }
}
