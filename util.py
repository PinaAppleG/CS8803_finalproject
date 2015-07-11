from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from optparse import OptionParser
import numpy as np

import pylab as pl

#Enable debugging
import pdb


def createList(path):
    f = open(path, "r")
    myList = []
    for line in f:
        entry = line.replace("\r\n", "").split(',')
        entry = [int(entry[0]), int(entry[1])]
        myList.append(entry)
        
    return myList

def testList(completeList):
    return completeList[:len(completeList)-60]

def expectedResult(completeList):
    return completeList[len(completeList)-60:]

def main():
    print "test"
    completeLists = []
    for i in range (1, 11):
        fileName = "test" + ("%02d" % (i,)) + ".txt"
        completeLists.append(createList("../inputs/" + fileName))
    actualResults = []
    for completeList in completeLists:
        actualResults.append(run(testList(completeLists)))
    scores = []
    for i in range(len(actualResults)):
        scores.append(compare(actualResults[i], expectedResult(completeLists[i])))
    score = sum(scores) / float(len(scores))
    print("Score for avg of all 10 was: " + str(score))

def compare(actual, expected):
    from math import sqrt
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

def run(measurements):
    # input is a list of lists of locations for every time frame
    # example: [[32,67],[35,67],[36,65],[36,64],[35,63]...]
    # output should be in the same form as input but contain only 60 frames
    output = []
    # enter code here
    output = filter(measurements)
    return output


def filter(measurements):
    #Mask the last 2 seconds of observations
    #for i in range(len(measurements)-10, len(measurements)):
    #    measurements[i] = np.ma.masked

    dt = 1.0
        
    x = np.array([[measurements[0][0]],
              [measurements[0][1]],
              [0.],
              [0.]])

    F = np.array([[1., 0., dt, 0.],
              [0., 0., 1., 0.],
              [0., 1., 0., dt],
              [0., 0., 0., 1.]])

    G = np.array([[0.5*dt**2],
                  [0.5*dt**2],
                  [dt],
                  [dt]])
    Q = G*G.T*0.1**2

    H = np.array([[1., 1., 0., 0.]])

    # Info available http://nbviewer.ipython.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/05_Multivariate_Kalman_Filters.ipynb
    bot_filter = KalmanFilter(dim_x=4, dim_z=2)
    bot_filter.x = x
    bot_filter.F = F
    bot_filter.H = np.asarray(H)
    bot_filter.Q = Q
    bot_filter.P *= 1000
    bot_filter.R = 1

    observable_meas = measurements[0:len(measurements)-60]

    pos, cov = [], []
    for z in observable_meas:
        pos.append(bot_filter.x)
        cov.append(bot_filter.P)
        
        bot_filter.update(z)
        bot_filter.predict()

    for i in range(0,60):
        bot_filter.predict()
        pos.append(bot_filter.x)
        
    return pos

#Parse command line options
parser = OptionParser()
parser.add_option("-f", "--file", dest="filename",
                  help="Input file of robot positions", metavar="FILE")
parser.add_option("-q", "--quiet",
                  action="store_false", dest="verbose", default=True,
                  help="don't print status messages to stdout")

(options, args) = parser.parse_args()

if options.filename:
    measurements = np.asarray(createList(options.filename))
    kf_out = run(measurements)
else:
    print "Must supply filename"


pl.figure(figsize=(16,6))

start = len(measurements)-60

true_measurements = np.asarray(createList(options.filename))


x_vals = [0.0]
y_vals = [0.0]

for i in range(1, len(kf_out)-1):
    x_vals.append(kf_out[i][0][0])
    y_vals.append(kf_out[i][0][1])

x_vals = np.asarray(x_vals)
y_vals = np.asarray(y_vals)

print kf_out[0:100]


#obs_scatter = pl.scatter(true_measurements[start:,0], true_measurements[start:,1], marker='x', color='r', label='observations')
kf_line = pl.plot(x_vals[start:], y_vals[start:], 'r--', true_measurements[start:,0], true_measurements[start:,1], 'bs')
pl.show()


