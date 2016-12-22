import ugfx

white = ugfx.color(255, 255, 255)
black = ugfx.color(0, 0, 0)


screen = ugfx.display("/dev/fb0")

c, d = black, white
x, w = 0, 400
while w >= 100:
    screen.box(x, x, x+w, x+w, c)
    x += 2
    w -= 4
    c, d = d, c

screen.invert(100, 100, 200, 200)
