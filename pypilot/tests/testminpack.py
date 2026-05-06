import numpy
import minpack
import vector

zpoints= [[-0.019627960233410714, 0.995653995871544, -0.9579334219296776, 0.08272908578316371, -0.05848887097090483, -0.027316996594890956, -0.009766031941398978, -0.026300387984762592], [-0.080971110034734, -0.17618845626711846, -0.032236056774854666, 0.9415856222311656, -0.08898376487195493, -0.10522267706692222, -0.056559245195239784, -1.019424915313721], [1.0010001134872437, -0.026635131915099922, 0.27431152363618216, 0.26205423921346666, -0.9751406878232955, -1.0298668444156647, -1.0282527506351473, -0.03473897281413277]]

#zpoints= [[1, 1, 1, 1, 0, 2], [-1, 1, 0, 0, 0, 0], [0, 0, 1, -1, 0, 0]]

beta0=[0, 0, 0, 1]

def lmap(*cargs):
    return list(map(*cargs))

def f_sphere3(beta, x):
    bias = beta[:3]
    b = numpy.matrix(lmap(lambda a, b : a - b, x[:3], bias))
    m = list(numpy.array(b.transpose()))
    r0 = lmap(lambda y : beta[3] - vector.norm(y), m)
    return r0

def minpack_f(beta, fvec):
    fvec[:] = f_sphere3(beta, zpoints)
    return 0

beta0 = numpy.array(beta0, dtype=float)
fvec = numpy.empty(len(zpoints[0]), dtype=float)

result = minpack.lmdif1(minpack_f, beta0, fvec)
if result in [1, 2, 3, 4]:
    print('minpack', beta0)

import odrpack

beta0= numpy.asarray([0, 0, 0, 1], dtype=float)
xdata = numpy.asarray(zpoints, dtype=float)
ydata = numpy.zeros(xdata.shape[-1], dtype=float)

def f_odrpack(x, beta):
    return numpy.asarray(f_sphere3(beta, x), dtype=float)

sol = odrpack.odr_fit(
    f_odrpack,
    xdata,
    ydata,
    beta0,
    task='implicit-ODR',
    maxit=1000,
)

#print("sol", sol)
if sol.success:
    print("odrpack", sol.beta)
