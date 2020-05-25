/* Copyright (C) 2020 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

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

    void refresh(int contrast, surface *s) {
        if(s->bypp != 1)
            return;

        int bias = 4;
        extended_command(PCD8544_SETBIAS | bias);
        contrast = contrast > 0x7f ? 0x7f : contrast;
        contrast = contrast < 0 ? 0 : contrast;
        extended_command(PCD8544_SETVOP | contrast);

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

