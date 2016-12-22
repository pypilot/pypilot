import ugfx
import font

white = ugfx.color(255, 255, 255)
black = ugfx.color(0, 0, 0)


screen = ugfx.display("/dev/fb0")
fonts = font.load('ugfxfonts', screen.bypp)

c, d = black, white
x, w = 0, 400
while w >= 100:
    screen.box(x, x, x+w, x+w, c)
    x += 2
    w -= 4
    c, d = d, c

font.draw(fonts[10], screen, "hello", 0, 20)
