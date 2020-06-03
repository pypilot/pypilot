/* Copyright (C) 2020 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

// JLX12864G-086


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
