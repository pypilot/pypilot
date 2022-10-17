/* Copyright (C) 2021 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */


#include <Arduino.h>

#include <SPI.h>

class JLX12864: public Print {
    public:
        // All the pins can be changed from the default values...
        JLX12864(unsigned char sclk  = 3,   /* clock       (display pin 2) */
                unsigned char sdin  = 4,   /* data-in     (display pin 3) */
                unsigned char dc    = 5,   /* data select (display pin 4) */
                unsigned char reset = 6,   /* reset       (display pin 8) */
                unsigned char sce   = 7);  /* enable      (display pin 5) */

        // Display initialization (dimensions in pixels)...
        void begin();

        // Write an ASCII character at the current cursor position (7-bit)...
#if ARDUINO < 100
        virtual void write(uint8_t chr);
#else
        virtual size_t write(uint8_t chr);
#endif
        void setfont(int font) { curfont = font; }

        void clear();
        void refresh(uint8_t page);
        void setpos(int x, int y) { xpos = x; ypos = y; }

        void putpixel(uint8_t x, uint8_t y, uint8_t color);
        void rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);
        void clear_lines(uint8_t y1, uint8_t y2);
        void circle(uint8_t x, uint8_t y, uint8_t r, uint8_t color);
        void line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);

        bool flip;
    private:
        void SetParameters();

        unsigned char pin_sclk;
        unsigned char pin_sdin;
        unsigned char pin_dc;
        unsigned char pin_reset;
        unsigned char pin_sce;

        SPISettings spi_settings;

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
