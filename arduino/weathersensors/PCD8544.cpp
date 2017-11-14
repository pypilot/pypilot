/*
 * PCD8544 - Interface with Philips PCD8544 (or compatible) LCDs.
 *
 * Copyright (c) 2010 Carlos Rodrigues <cefrodrigues@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#include "PCD8544.h"

#include <HardwareSerial.h>

void debug(char *fmt, ... ){
    char buf[128]; // resulting string limited to 128 chars
    va_list args;
    va_start (args, fmt );
    vsnprintf(buf, 128, fmt, args);
    va_end (args);
    Serial.print(buf);
}

#if ARDUINO < 100
#include <WProgram.h>
#else
#include <Arduino.h>
#endif

#include <SPI.h>

#include "charset.h"
#include <avr/pgmspace.h>


#define PCD8544_CMD  LOW
#define PCD8544_DATA HIGH

#define HWSPI // hardware spi is 12x faster


/*
 * If this was a ".h", it would get added to sketches when using
 * the "Sketch -> Import Library..." menu on the Arduino IDE...
 */
//#include "charset.cpp"


static unsigned char framebuffer[6*84];

PCD8544::PCD8544(unsigned char sclk, unsigned char sdin,
                 unsigned char dc, unsigned char reset,
                 unsigned char sce):
    pin_sclk(sclk),
    pin_sdin(sdin),
    pin_dc(dc),
    pin_reset(reset),
    pin_sce(sce)
{}


void PCD8544::begin(unsigned char width, unsigned char height, unsigned char model)
{
    this->width = width;
    this->height = height;

    // Only two chip variants are currently known/supported...
    this->model = (model == CHIP_ST7576) ? CHIP_ST7576 : CHIP_PCD8544;

    // All pins are outputs (these displays cannot be read)...
    pinMode(this->pin_sclk, OUTPUT);
    pinMode(this->pin_sdin, OUTPUT);
    pinMode(this->pin_dc, OUTPUT);
    pinMode(this->pin_reset, OUTPUT);
    if(this->pin_sce != 99)
        pinMode(this->pin_sce, OUTPUT);

#ifdef HWSPI
    SPI.begin();
#endif
    // Reset the controller state...
    digitalWrite(this->pin_reset, HIGH);
    digitalWrite(this->pin_sce, HIGH);
    digitalWrite(this->pin_reset, LOW);
    delay(100);
    digitalWrite(this->pin_reset, HIGH);

    // Set the LCD parameters...
    this->send(PCD8544_CMD, 0x21);  // extended instruction set control (H=1)
    this->send(PCD8544_CMD, 0x13);  // bias system (1:48)

    if (this->model == CHIP_ST7576) {
        this->send(PCD8544_CMD, 0xe0);  // higher Vop, too faint at default
        this->send(PCD8544_CMD, 0x05);  // partial display mode
    } else {
        this->send(PCD8544_CMD, 0xc2);  // default Vop (3.06 + 66 * 0.06 = 7V)
    }

    this->send(PCD8544_CMD, 0x20);  // extended instruction set control (H=0)
    this->send(PCD8544_CMD, 0x09);  // all display segments on

    // Clear RAM contents...
    this->clear();

    // Activate LCD...
    this->send(PCD8544_CMD, 0x08);  // display blank
    this->send(PCD8544_CMD, 0x0c);  // normal mode (0x0d = inverse mode)
    delay(100);

    // Place the cursor at the origin...
    this->send(PCD8544_CMD, 0x80);
    this->send(PCD8544_CMD, 0x40);
}


void PCD8544::stop()
{
    this->clear();
    this->setPower(false);
}

void PCD8544::setPower(bool on)
{
    this->send(PCD8544_CMD, on ? 0x20 : 0x24);
}


inline void PCD8544::display()
{
    this->setPower(true);
}


inline void PCD8544::noDisplay()
{
    this->setPower(false);
}


void PCD8544::setInverse(bool inverse)
{
    this->send(PCD8544_CMD, inverse ? 0x0d : 0x0c);
}


void PCD8544::setContrast(unsigned char level)
{
    // The PCD8544 datasheet specifies a maximum Vop of 8.5V for safe
    // operation in low temperatures, which limits the contrast level.
    if (this->model == CHIP_PCD8544 && level > 90) {
        level = 90;  // Vop = 3.06 + 90 * 0.06 = 8.46V
    }

    // The ST7576 datasheet specifies a minimum Vop of 4V.
    if (this->model == CHIP_ST7576 && level < 36) {
        level = 36;  // Vop = 2.94 + 36 * 0.03 = 4.02V
    }

    this->send(PCD8544_CMD, 0x21);  // extended instruction set control (H=1)
    this->send(PCD8544_CMD, 0x80 | (level & 0x7f));
    this->send(PCD8544_CMD, 0x20);  // extended instruction set control (H=0)
}

void PCD8544::putpixel(uint8_t x, uint8_t y, uint8_t color)
{
    if(x < 0 || x >= 48 || y < 0 || x >= 84)
        return;
    x = 48 - x;
    int col = x/8, bit = 1<<(x%8);
    int ind = col*84 + y;
    if(color)
        framebuffer[ind] |= bit;
    else
        framebuffer[ind] &= ~bit;
}

void PCD8544::rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color)
{
    for(uint8_t y=y1; y<=y2; y++)
        for(uint8_t x=x1; x<=x2; x++)
            putpixel(x, y, color);
}

void PCD8544::circle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color)
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

void PCD8544::line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color)
{
    int dx = abs(x2-x1), dy = abs(y2-y1);

    if(dy > dx)
       DRAW_LINE(x, y);
    else
       DRAW_LINE(y, x);
}


void PCD8544::setCursor(unsigned char column, unsigned char line)
{
    column = (column % this->width);
    line = (line % (this->height/9 + 1));

    this->send(PCD8544_CMD, 0x80 | column);
    this->send(PCD8544_CMD, 0x40 | line);
}


#if ARDUINO < 100
void PCD8544::write(uint8_t chr)
#else
size_t PCD8544::write(uint8_t chr)
#endif
{
    // ASCII 7-bit only...
    if (chr >= 0x80) {
#if ARDUINO < 100
        return;
#else
        return 0;
#endif
    }

    struct font f;
    memcpy_P(&f, fonts + curfont, sizeof f);
    struct font_character c;
    for(int i=0; i<f.n; i++) {
        memcpy_P(&c, f.characters + i, sizeof c);
        if(c.c == chr)
            goto found;
    }
    return 0; // not found
found:
    {
        unsigned char glyph[c.w*c.h];
        memcpy_P(glyph, c.data, c.w*c.h);

    for(int y=0; y<c.h; y++) {
        int yp = ypos + y;
        if(yp < 0 || yp >= 84)
            continue;
        for(int x=0; x<c.w; x++) {
            if(!glyph[y*c.w + x])
                continue;

            int xp = xpos + x;
            if(xp < 0 || xp >= 48)
                continue;
            putpixel(xp, yp, 255);
        }
    }
    xpos += c.w + 1;
    }

#if ARDUINO >= 100
    return 1;
#endif
}


void PCD8544::send(unsigned char type, unsigned char data)
{
    if(this->lasttype != type) {
        digitalWrite(this->pin_dc, type);
        this->lasttype = type;
    }
    
    if(this->pin_sce != 99)
        digitalWrite(this->pin_sce, LOW);
#ifdef HWSPI
    SPI.transfer(data);
#else
    shiftOut(this->pin_sdin, this->pin_sclk, MSBFIRST, data);
#endif
    if(this->pin_sce != 99)
        digitalWrite(this->pin_sce, HIGH);
}

void PCD8544::clear()
{
    memset(framebuffer, 0, sizeof framebuffer);
}

void PCD8544::refresh()
{
    setCursor(0, 0);
    digitalWrite(this->pin_dc, PCD8544_DATA);
    for(unsigned int i=0; i<sizeof framebuffer; i++) {
        SPDR = framebuffer[i];
        while (!(SPSR & _BV(SPIF)));
    }
    this->lasttype = PCD8544_DATA;
}

