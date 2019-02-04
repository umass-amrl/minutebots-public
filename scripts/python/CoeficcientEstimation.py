#!/usr/bin/env python3

import matplotlib.pyplot as plt



def coefficient(x,y):
    x_1 = x[0]
    x_2 = x[1]
    x_3 = x[2]
    y_1 = y[0]
    y_2 = y[1]
    y_3 = y[2]

    a = y_1/((x_1-x_2)*(x_1-x_3)) + y_2/((x_2-x_1)*(x_2-x_3)) + y_3/((x_3-x_1)*(x_3-x_2))

    b = (-y_1*(x_2+x_3)/((x_1-x_2)*(x_1-x_3))
         -y_2*(x_1+x_3)/((x_2-x_1)*(x_2-x_3))
         -y_3*(x_1+x_2)/((x_3-x_1)*(x_3-x_2)))

    c = (y_1*x_2*x_3/((x_1-x_2)*(x_1-x_3))
        +y_2*x_1*x_3/((x_2-x_1)*(x_2-x_3))
        +y_3*x_1*x_2/((x_3-x_1)*(x_3-x_2)))

    return a,b,c

x = [0,1,2]
y = [0,1,4]

a,b,c = coefficient(x, y) 

print ("a = {}".format(a))
print ("b = {}".format(b))
print ("c = {}".format(c))
print ("{}x^2 + {}x + {}".format(a, b, c))

def evaluate(a, b, c, x):
    return a * x**2 + b * x + c

xs = [x for x in range(10)]
ys = [evaluate(a, b, c, x) for x in xs]

plt.plot(xs, ys)
plt.title("Power vs Distance plot\n${}x^2 + {}x + {}$".format(a, b, c))
plt.show()

