#!/usr/bin/env python3

for i in range(12):
    file_name = "route{}.txt".format(i)
    f  = open(file_name, "w")
    print(file_name)
    x_lower = 1500
    x_upper = 2500
    y_lower = i * 300 + 300
    y_upper = i * 300
    f.write("{} {}\n".format(x_lower, y_lower))
    f.write("{} {}\n".format(x_upper, y_upper))
    f.close()
