from math import sqrt
from collections import Counter

from filterpy.kalman import KalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.common import Q_discrete_white_noise

from optparse import OptionParser

import pylab as pl

#Enable debugging
import pdb

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import leastsq


def createList(path):
    f = open(path, "r")
    myList = []
    for line in f:
        myList.append(line.replace("\n", "").split(','))
    myIntLists = []
    for i in myList:
        myIntLists.append([int(i[0]), int(i[1])])
    return myIntLists

def inBounds(coordinates):
    # These numbers are based on the avg of the 10 tests
    return (coordinates[0] > 250 and coordinates[0] < 1680 and coordinates[1] > 117 and coordinates[1] < 971)

def printBounds(path):
    maxX = 0
    maxY = 0
    minX = 100000
    minY = 100000
    for i in createList(path):
      if(i[0] > maxX):
        maxX = i[0]
      if(i[0] < minX):
        minX = i[0]
      if(i[1] > maxY):
        maxY = i[1]
      if(i[1] < minY):
        minY = i[1]
    print("maxX: " + str(maxX))
    print("maxY: " + str(maxY))
    print("minX: " + str(minX))
    print("minY: " + str(minY))

def testList(completeList):
    return completeList[:len(completeList)-60]

def expectedResult(completeList):
    return completeList[len(completeList)-60:]

def main():
    completeLists = []
    for i in range (1, 11):
        fileName = "test" + ("%02d" % (i,)) + ".txt"
        completeLists.append(createList("inputs/" + fileName))
    actualResults = []
    for completeList in completeLists:
        actualResults.append(run(testList(completeList)))
    scores = []
    for i in range(len(actualResults)):
        scores.append(compare(actualResults[i], expectedResult(completeLists[i])))
    print scores
    scores.remove(max(scores))
    scores.remove(min(scores))
    score = sum(scores) / float(len(scores))
    print("Score for avg of all 10 was: " + str(score))

def compare(actual, expected):
    frames = 60
    if(len(actual) != frames or len(expected) != frames):
        raise ValueError("Sizes of actual/expected were not " + str(frames) + ": " + str(len(actual)) + "/" + str(len(expected)))
    runningSum = 0
    for i in range(frames):
        distX = abs(int(actual[i][0]) - int(expected[i][0]))
        distY = abs(int(actual[i][1]) - int(expected[i][1]))
        dist = sqrt(distX ** 2 + distY ** 2)
        runningSum += dist ** 2
    return sqrt(runningSum)


def changeInDist(p1, p2):
    return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def run(input):
    # input is a list of lists of locations for every time frame
    # example: [[32,67],[35,67],[36,65],[36,64],[35,63]...]
    # output should be in the same form as input but contain only 60 frames
    output = []
    # lists will contain all of the lists of data we have, including training data
    # and test input data
    lists = []
    # add a list of training data
    lists.append(createList("inputs/" + "training_data.txt"))
    # add the ten input lists
    for i in range (1, 11):
        fileName = "test" + ("%02d" % (i,)) + ".txt"
        lists.append(testList(createList("inputs/" + fileName)))
    # listId will store the index in lists of the list containing the best match
    listId = 0
    # bestMatch will contain the index in the list of the best match
    # e.g. the best match could be at lists[listId][bestMatch]
    bestMatch = -1
    # matchDiff is the difference in distance between the current best match and the input
    matchDiff = 1000
    # N is the number of points that we are going to compare from the end of the input to the particles
    N = 6
    # loop through all of the lists
    for f in range(len(lists)):
        # training will hold the current list
        training = lists[f]
        # inputPoints will be the last N points in the input
        inputPoints = input[len(input)-N:]
        # now we loop through the training list looking for the best match
        # we start N points into the list and end when there are 60 points left
        # because we compare N consecutive points and need 60 points to return
        for i in range(N-1,len(training)-60):
            # test contains the N points that we will compare to the N points at the end of the input
            test = training[i-N+1:i+1]
            # these are the velocities in the x and y directions between the last two points of the test data
            vxtest = test[N-1][0] - test[N-2][0]
            vytest = test[N-1][1] - test[N-2][1]
            # these are the velocities in the x and y directions between the last two points of the input data
            vxinput = inputPoints[N-1][0] - inputPoints[N-2][0]
            vyinput = inputPoints[N-1][1] - inputPoints[N-2][1]
            # first check that the velocities of the input and test have the same signs
            if (vxtest<0) == (vxinput<0) and (vytest<0) == (vyinput<0):
                # totalDiff is the total change in distance between the input and test N points
                totalDiff = 0
                for n in range(N):
                    totalDiff += changeInDist(inputPoints[n], test[n])
                # if we found a better match than the currently saved match, update our variables for the new match
                if totalDiff < matchDiff:
                    listId = f
                    bestMatch = i
                    matchDiff = totalDiff
    xdiff = []
    ydiff = []
    # loop through the 60 subsequent points after the best match point and calculate
    # the difference in x and y between each consecutive point, pushing those values
    # to xdiff and ydiff respectively
    for i in range(bestMatch+1, bestMatch+61):
        prev = lists[listId][i-1]
        thisOne = lists[listId][i]
        xdiff.append(thisOne[0] - prev[0])
        ydiff.append(thisOne[1] - prev[1])
    # loop through xdiff and ydiff, and apply those motions to the last point in the input,
    # appending those 60 values to the output to return
    prev = input[len(input)-1]
    for i in range(len(xdiff)):
        output.append([prev[0] + xdiff[i], prev[1] + ydiff[i]])
        prev = output[len(output) - 1]

    print matchDiff, "            ", listId
    # example perfect output from test01.txt:
    # output = [['1153', '896'], ['1156', '923'], ['1163', '934'], ['1159', '930'], ['1159', '910'], ['1154', '925'], ['1150', '921'], ['1152', '924'], ['1152', '913'], ['1152', '925'], ['1161', '919'], ['1155', '934'], ['1163', '927'], ['1164', '930'], ['1163', '930'], ['1155', '953'], ['1148', '951'], ['1141', '944'], ['1136', '944'], ['1120', '949'], ['1120', '946'], ['1110', '946'], ['1104', '946'], ['1090', '949'], ['1074', '948'], ['1053', '946'], ['1027', '938'], ['1006', '931'], ['993', '926'], ['977', '918'], ['963', '910'], ['938', '897'], ['920', '886'], ['903', '879'], ['880', '865'], ['865', '858'], ['845', '846'], ['821', '834'], ['801', '826'], ['776', '816'], ['751', '810'], ['726', '803'], ['704', '796'], ['679', '790'], ['654', '782'], ['632', '773'], ['613', '761'], ['593', '749'], ['575', '737'], ['560', '728'], ['543', '714'], ['525', '701'], ['507', '688'], ['492', '671'], ['478', '660'], ['465', '645'], ['453', '631'], ['440', '616'], ['426', '594'], ['418', '579']]
    return output

def graph(filenum):
    filename = "inputs/test" + filenum + ".txt"
    if filename:
        measurements = testList(createList(filename))
        kf_out = run(measurements)
    else:
        print "Must supply filename"


    pl.figure(figsize=(16,6))

    start = len(measurements)-65
    finish = len(measurements)-40

    true_measurements = np.asarray(createList(filename))

    x_vals = []
    y_vals = []

    for i in range(len(kf_out)):
        x_vals.append(kf_out[i][0])
        y_vals.append(kf_out[i][1])

    x_vals = np.asarray(x_vals)
    y_vals = np.asarray(y_vals)

    obs_scatter = pl.scatter(true_measurements[start:,0], true_measurements[start:,1], marker='x', color='r', label='observations')
    kf_line = pl.plot(x_vals, y_vals, 'r--', true_measurements[start:,0], true_measurements[start:,1], 'bs')
    pl.show()


#main()

graph("08")

def test():
    completeLists = []
    # for i in range (1, 11):
    #     fileName = "test" + ("%02d" % (i,)) + ".txt"
    #     completeLists.append(createList("inputs/" + fileName))
    #completeLists.append(createList("inputs/" + "training_data.txt"))
    completeLists.append(createList("inputs/" + "test01.txt"))
    dists = []
    x = []
    y = []
    v = []
    a = []
    for completeList in completeLists:
        distsTemp, xTemp, yTemp, vTemp, aTemp = averageChangeInDist(completeList)
        dists.extend(distsTemp)
        x.extend(xTemp)
        y.extend(yTemp)
        v.extend(vTemp)
        a.extend(aTemp)
    # print "x: ", x
    # print sum(x)/float(len(x))
    # print "y: ", y
    # print sum(y)/float(len(y))
    # print "v: ", v
    # print sum(v)/float(len(v))
    # print "dist: ", dists
    # print sum(dists)/float(len(dists))
    t = np.array(range(len(y)))
    Y = np.array(y)
    z = np.polyfit(t,Y, 50)
    p = np.poly1d(z)
    py = []
    for i in t:
        py.append(p(i))
    plt.plot(t,Y,'r',t,py,'g')
    plt.show()

    vCount = Counter(v)
    print "v: ", Counter(v)
    print "a: ", Counter(a)
    print "x: ", Counter(x)
    print "y: ", Counter(y)
    print "dists: ", Counter(dists)
    return dists, x, y, v, a

def averageChangeInDist(l):
    dists = []
    x = []
    y = []
    v = []
    a = []
    for i in range(1,len(l)):
        dists.append(changeInDist(l[i], l[i-1]))
        x.append(l[i][0]-l[i-1][0])
        y.append(l[i][1]-l[i-1][1])
        if (x[i-1] != 0):
            v.append(y[i-1]/x[i-1])
        # else:
        #     v.append("u")
    for i in range(1, len(v)):
        if (v[i-1] not in ["u"] and v[i] not in ["u"]):
            a.append(v[i] - v[i-1])
        # else:
        #     a.append("u")
    # print "x: ", x
    # print sum(x)/float(len(x))
    # print "y: ", y
    # print sum(y)/float(len(y))
    # print "v: ", v
    # print sum(v)/float(len(v))
    # print "dist: ", dists
    # print sum(dists)/float(len(dists))
    return dists, x, y, v, a


#test()

# l = createList("inputs/" + "test01.txt")
# x = []
# y = []
# t = []
# for i in range(len(l)):
#     x.append(l[i][0])
#     y.append(l[i][1])
#     t.append(i)
# x2 = []
# y2 = []
# t2 = []
# for i in range(len(l)):
#     x2.append(l[i][0])
#     y2.append(l[i][1])
#     t2.append(i)
# T = np.array(t)
# Y = np.array(y)
# z = np.polyfit(T,Y, 50)
# p = np.poly1d(z)
# T2 = np.array(t2)
# Y2 = np.array(y2)
# py = []
# for i in t2:
#     py.append(p(i))
# plt.plot(T2,Y2,'r',T2,py,'g')
# plt.show()