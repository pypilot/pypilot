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


#ifndef PCD8544_H
#define PCD8544_H


#if ARDUINO < 100
#include <WProgram.h>
#else
#include <Arduino.h>
#endif


// Chip variants supported (ST7576 is experimental)...
#define CHIP_PCD8544 0
#define CHIP_ST7576  1


class PCD8544: public Print {
    public:
        // All the pins can be changed from the default values...
        PCD8544(unsigned char sclk  = 3,   /* clock       (display pin 2) */
                unsigned char sdin  = 4,   /* data-in     (display pin 3) */
                unsigned char dc    = 5,   /* data select (display pin 4) */
                unsigned char reset = 6,   /* reset       (display pin 8) */
                unsigned char sce   = 7);  /* enable      (display pin 5) */

        // Display initialization (dimensions in pixels)...
        void begin(unsigned char width=84, unsigned char height=48, unsigned char model=CHIP_PCD8544);
        void stop();

        // Control the display's power state...
        void setPower(bool on);

        // For compatibility with the LiquidCrystal library...
        void display();
        void noDisplay();

        // Activate white-on-black mode (whole display)...
        void setInverse(bool inverse);

        // Set display contrast level (0-127)...
        void setContrast(unsigned char level);

        // Place the cursor at position (column, line)...
        void setCursor(unsigned char column, unsigned char line);

        // Write an ASCII character at the current cursor position (7-bit)...
#if ARDUINO < 100
        virtual void write(uint8_t chr);
#else
        virtual size_t write(uint8_t chr);
#endif
        void setfont(int font) { curfont = font; }

        void clear();
        void refresh();
        void setpos(int x, int y) { xpos = x; ypos = y; }

        void putpixel(uint8_t x, uint8_t y, uint8_t color);
        void rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);
        void circle(uint8_t x, uint8_t y, uint8_t r, uint8_t color);
        void line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);

    private:
        unsigned char pin_sclk;
        unsigned char pin_sdin;
        unsigned char pin_dc;
        unsigned char pin_reset;
        unsigned char pin_sce;

        // Chip variant in use...
        unsigned char model;

        // The size of the display, in pixels...
        unsigned char width;
        unsigned char height;

        unsigned char lasttype;

        // Send a command or data to the display...
        void send(unsigned char type, unsigned char data);

        int xpos, ypos;
        int curfont;
};


#endif  /* PCD8544_H */


/* vim: set expandtab ts=4 sw=4: */
