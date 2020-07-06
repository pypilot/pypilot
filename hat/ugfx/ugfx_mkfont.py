import os, sys
d = '/home/sean/.pypilot/ugfxfonts/'
files = os.listdir(d)

fonts = {}
for f in files:
    fn = f[:3]
    if not fn in fonts:
        fonts[fn] = {}
    font = fonts[fn]
    cn = f[3:]
    file = open(d+f, 'rb')
    font[cn] = file.read()
    file.close()

print('struct character {')
print('    int16_t len;')
print('    const char *cn;')
print('    const char *data;')
print('};')
print('')

for fn in fonts:
    for cn in fonts[fn]:
        print('const char data' + fn + cn + '[] PROGMEM = {')
        for c in fonts[fn][cn]:
            sys.stdout.write('0x%x, ' % c)
        print('};')
        print('const char cn'+fn+cn+'[] PROGMEM = "' + cn + '";')
        print('')

print('struct font {')
print('   const char *fn;')
print('   int16_t count;')
print('   const struct character *characters;')
print('};')
print('')
    
for fn in fonts:
    print('const char fn'+fn+'[] PROGMEM = "' + fn + '";')
    print('const struct character characters'+fn+'[] PROGMEM = {')
    for cn in fonts[fn]:
        print('{'+str(len(fonts[fn][cn]))+', cn'+fn+cn + ', data'+fn+cn+'}, ')
    print('};')
    print('const struct font font'+fn+'={fn'+fn+', ' + str(len(fonts[fn])) + ', characters'+fn+'};')
    print('')

print('const struct font *fonts[] PROGMEM = {')
for fn in fonts:
    print('    &font'+fn+',')
print('};')
