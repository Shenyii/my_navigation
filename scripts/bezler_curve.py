#coding: utf-8
import numpy
import matplotlib.pyplot as plt
import math

def bezlerCurve():
    x0 = [0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4]
    y0 = [0, 1, 1, 1, 1, 1, 1, 1, 0]
    x = []
    y = []
    n = len(x0)
    for t in numpy.arange(0, 1.02, 0.02):
        x1 = 0
        y1 = 0
        for i in range(len(x0)):
            x1 += (math.factorial(n-1) / math.factorial(i) / math.factorial(n-1 - i) * x0[i] * pow(1-t, n-1-i) * pow(t, i))
            y1 += (math.factorial(n-1) / math.factorial(i) / math.factorial(n-1 - i) * y0[i] * pow(1-t, n-1-i) * pow(t, i))
        x.append(x1)
        y.append(y1)

    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.plot(x, y, '-')
    plt.show()

if __name__ == '__main__':
    print('bezler curve.')
    bezlerCurve()