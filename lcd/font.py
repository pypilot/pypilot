#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from __future__ import print_function
import os
from ugfx import ugfx

global fonts
fonts = {}

fontpath = os.path.abspath(os.getenv('HOME') + '/.pypilot/ugfxfonts/')

                                                                                                    

if not os.path.exists(fontpath):
    os.makedirs(fontpath)
if not os.path.isdir(fontpath):
    raise 'ugfxfonts should be a directory'


def draw(surface, pos, text, size, bw, crop=False):
    if not size in fonts:
        fonts[size] = {}

    font = fonts[size]
    if pos:
        x, y = pos
    else:
        x, y = 0, 0

    origx = x
    width = 0
    lineheight = 0
    height = 0
    for c in text:
        if c == '\n':
            x = origx
            y += lineheight
            height += lineheight
            lineheight = 0
            continue
        
        if not c in font:
            filename = fontpath + '/%03d%03d' % (size, ord(c))
            if bw:
                filename += 'b';
            if crop:
                filename += 'c';

            #print('ord', ord(c), filename)
            font[c] = ugfx.surface(filename)
            if font[c].bypp != surface.bypp:
                font[c] = create_character(os.path.abspath(os.path.dirname(__file__)) + "/font.ttf", size, c, surface.bypp, crop, bw)
                font[c].store_grey(filename)

        if pos:
            surface.blit(font[c], x, y)
        x += font[c].width
        width = max(width, x-origx)
        lineheight = max(lineheight, font[c].height)

    return width, height+lineheight

def create_character(fontpath, size, c, bypp, crop, bpp):
    try:
        from PIL import Image
        from PIL import ImageDraw
        from PIL import ImageFont
        from PIL import ImageChops

    except:
        # we will get respawn hopefully after python-PIL is loaded
        print('failed to load PIL to create fonts, aborting...')
        import time
        time.sleep(3)
        exit(1)

    ifont = ImageFont.truetype(fontpath, size)
    size = ifont.getsize(c)
    image = Image.new('RGBA', size)
    draw = ImageDraw.Draw(image)
    draw.text((0, 0), c, font=ifont)

    if crop:
        bg = Image.new(image.mode, image.size, image.getpixel((0, 0)))
        diff = ImageChops.difference(image, bg)
        bbox = diff.getbbox()
        if bbox:
            image = image.crop(bbox)

    if bpp:
        data = list(image.getdata())
        for i in range(len(data)):
            d = 255 / (1<<bpp)
            v = int(round(data[i][0] / (255 / (1<<bpp))) * (255 / ((1<<bpp)-1)))
            data[i] = (v,v,v,v)
        image.putdata(data)
    return ugfx.surface(image.size[0], image.size[1], bypp, image.tobytes())
    
if __name__ == '__main__':
    print('ugfx test program')
    screen = ugfx.display("/dev/fb0")
    draw(screen, (0, 100), "1234567890", 28);
