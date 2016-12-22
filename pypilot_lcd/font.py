import pickle
import ctypes
import ugfx

def load(filename, bypp):
    data = pickle.load(open(filename, 'rb'))
    fonts = {}
    for s in data:
        fonts[s] = {}
        for c in data[s]:
            d = data[s][c]
            count = d['size'][0]*d['size'][1]*4
            bytes = d['data']

            fonts[s][c] = ugfx.surface(d['size'][0], d['size'][1], bypp, bytes)
    return fonts

def draw(surface, pos, text, font):
    x, y = pos
    for c in text:
        if c in font:
            surface.blit(font[c], x, y)
            x += font[c].width

if __name__ == '__main__':
    import ctypes
    from PIL import Image
    from PIL import ImageDraw
    from PIL import ImageFont
    from PIL import ImageChops

    def create(filename, fontpath, size, alpha=False, symbol=False):
        ifont = ImageFont.truetype(fontpath, size)
        font = {}

        
        numbers = '0123456789'
        numeric = numbers + '.+- '
        letters = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ' + \
                  'abcdefghijklmnopqrstuvwxyz'
        symbols = '`~!@#$%^&*()+=,/;\'<>?:"[{}]'
        
        chars = numeric
        if alpha:
            chars += letters
        if symbol:
            chars += symbols
        for c in chars:
            size = ifont.getsize(c)
            image = Image.new('RGBA', size)
            draw = ImageDraw.Draw(image)
            draw.text((0, 0), c, font=ifont)

            def trim(im):
                bg = Image.new(im.mode, im.size, im.getpixel((0, 0)))
                diff = ImageChops.difference(im, bg)
                bbox = diff.getbbox()
                if bbox:
                    return im.crop(bbox)

            if c in numbers and not alpha:
                image = trim(image)

            if image:
                font[c] = {'size' : image.size, 'data' : image.tobytes()} 

        return font

    fonts = {}
    for s in [8, 10, 12, 16, 28]:
        fonts[s] = create('font'+str(s), 'font.ttf', s, s < 16)

    pickle.dump(fonts, open('ugfxfonts', 'wb'))

    data = fonts
    screen = ugfx.display("/dev/fb0")

    fonts = {}
    for s in data:
        fonts[s] = {}
        for c in data[s]:
            d = data[s][c]
            count = d['size'][0]*d['size'][1]*4
            bytes = d['data']

            fonts[s][c] = ugfx.surface(d['size'][0], d['size'][1], screen.bypp, bytes)

    draw(screen, (0, 100), "1234567890", fonts[28])
