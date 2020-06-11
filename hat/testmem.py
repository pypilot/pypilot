import time
t0 = time.time()
f = open('ugfxfonts/086050c')
print('f',f)
v = f.read(100)
print('dd', len(v))
f.close()
print('time', time.time()-t0)
