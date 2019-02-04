#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import sys

def extractDatum(file):
    accelerations = [] 
    line = file.readline()
    SIndex = line.find('(')
    MIndex = line.find(',')
    EIndex = line.find(')')
    x = float(line[SIndex + 1 : MIndex])
    y = float(line[MIndex + 2 : EIndex])
    vel = [x, y]

    line = file.readline()
    SIndex = line.find('(')
    MIndex = line.find(',')
    EIndex = line.find(')')
    x = float(line[SIndex + 1 : MIndex])
    y = float(line[MIndex + 2 : EIndex])
    pos = [x, y]

    time = file.readline() #not currently used;
    junk = file.readline()
    junk = file.readline()

    while (junk[0] != '-'):
        line = file.readline()
        SIndex = line.find('(')
        MIndex = line.find(',')
        EIndex = line.find(')')
        x = float(line[SIndex + 1 : MIndex])
        y = float(line[MIndex + 2 : EIndex])
        accel = [x, y]
        line = file.readline()
        SIndex = line.find('(')
        EIndex = line.find(')')
        time = float(line[SIndex + 1 : EIndex])
        accelTime = [accel, time]
        accelerations.append(accelTime)
        junk = file.readline()

    junk = file.readline()
    return [vel, pos, accelerations]


def getData(file):
    points = []
    point = extractDatum(file)
    while (len(point[2]) > 0):
        points.append(point)
        point = extractDatum(file)
    points.append(point)
    return points


def getAccelPoints(data):
    xData = [data[1][0]]
    yData = [data[1][1]]
    xVel = data[0][0]
    yVel = data[0][1]
    for i in xrange(0, len(data[2])):
        accel = data[2][i][0]
        time = data[2][i][1]
        lastX = xData[len(xData) - 1]
        lastY = yData[len(yData) - 1]
        x = lastX + xVel * time + accel[0] * time * time / 2
        xVel += accel[0] * time
        y = lastY + yVel * time + accel[1] * time * time / 2
        yVel += accel[1] * time
        xData.append(x)
        yData.append(y)
    return [xData, yData]

def allPoints(data):
    allData = []
    for i in xrange(0, len(data)):
        allData.append(getAccelPoints(data[i]))
    return allData

def getActualMotion(data):
    xData = []
    yData = []
    for i in xrange(0,len(data)):
        xData.append(data[i][1][0])
        yData.append(data[i][1][1])
    return [xData, yData]

def plotDatum(data, myLine, myColor, myAlpha):
    plt.scatter(data[0], data[1], color=myColor, alpha = myAlpha)
    for i in xrange(1,len(data[0])):
        plt.plot([data[0][i - 1], data[0][i]], [data[1][i - 1], data[1][i]], linestyle = myLine, color=myColor, alpha=myAlpha)

def plotData(data, alpha):
    colors = ['b', 'g', 'r', 'y']
    for i in xrange(0, len(data)):
        myColor = colors[i % len(colors)]
        plotDatum(data[i], 'dotted', myColor, alpha)

def driver(fileName):
    f = open(fileName, 'r')
    data = getData(f)
    motion = getActualMotion(data)
    potential = allPoints(data)
    plotData(potential, .4)
    plotDatum(motion, 'solid', 'black', .4)
    plt.show()
    f.close()

if __name__ == "__main__":
    driver(sys.argv[1])
    


