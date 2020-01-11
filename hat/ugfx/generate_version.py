#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from PIL import Image

import pypilot.version

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont


ifont = ImageFont.truetype('../font.ttf', 14)
version = "0.14"#pypilot.version.strversion
size = ifont.getsize(version)
image = Image.new('RGBA', size)
draw = ImageDraw.Draw(image)
draw.text((0, 0), version, font=ifont)

data = list(image.getdata())
print('len', len(data), size)

f = open('pypilot_version.h', 'w')
f.write('static unsigned int width = %d;\n' % size[0])
f.write('static unsigned int height = %d;\n\n' % size[1])

f.write('#define HEADER_PIXEL(data,pixel) {\\\n')
f.write('pixel[0] = data[0]; \\\n')
f.write('pixel[1] = data[0]; \\\n')
f.write('pixel[2] = data[0]; \\\n')
f.write('data ++; }\n\n')
f.write('static char header_data[] = {')


for i in range(len(data)):
    d = data[i]
    if d[0] < 128:
        f.write('255,')
    else:
        f.write('0,')

f.write('};\n')
f.close()
