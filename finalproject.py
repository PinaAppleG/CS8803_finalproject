from math import sqrt
import sys

# write list of points to prediction.txt
def writeToFile(measurements):
    if(len(measurements) != 60):
        print("WRONG SIZE BEING PRINTED!")
    f = open('prediction.txt', 'w')
    for measurement in measurements:
        f.write(str(measurement[0]) + "," + str(measurement[1]) + "\n")
    f.close()

# take path to file, read file, and return list of points in file
def createList(path):
    f = open(path, "r")
    myList = []
    for line in f:
        entry = line.replace("\r\n", "").split(',')
        entry = [int(entry[0]), int(entry[1])]
        myList.append(entry)
    return myList

# take in list and return list minus 60 last points off the end
def testList(completeList):
    return completeList[:len(completeList)-60]

# return the distance between p1 and p2
def changeInDist(p1, p2):
    return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

# input is the test file as a list, run our algorithm to create prediction, and return 60 points as a list
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
    # example perfect output from test01.txt:
    # output = [['1153', '896'], ['1156', '923'], ['1163', '934'], ['1159', '930'], ['1159', '910'], ['1154', '925'], ['1150', '921'], ['1152', '924'], ['1152', '913'], ['1152', '925'], ['1161', '919'], ['1155', '934'], ['1163', '927'], ['1164', '930'], ['1163', '930'], ['1155', '953'], ['1148', '951'], ['1141', '944'], ['1136', '944'], ['1120', '949'], ['1120', '946'], ['1110', '946'], ['1104', '946'], ['1090', '949'], ['1074', '948'], ['1053', '946'], ['1027', '938'], ['1006', '931'], ['993', '926'], ['977', '918'], ['963', '910'], ['938', '897'], ['920', '886'], ['903', '879'], ['880', '865'], ['865', '858'], ['845', '846'], ['821', '834'], ['801', '826'], ['776', '816'], ['751', '810'], ['726', '803'], ['704', '796'], ['679', '790'], ['654', '782'], ['632', '773'], ['613', '761'], ['593', '749'], ['575', '737'], ['560', '728'], ['543', '714'], ['525', '701'], ['507', '688'], ['492', '671'], ['478', '660'], ['465', '645'], ['453', '631'], ['440', '616'], ['426', '594'], ['418', '579']]
    return output

# main function that takes in the path to the file, calls the functions to create a list out of the file,
# run our prediction algorithm on the list, and write the prediction to a file
def main(filename):
    measurements = createList(filename)
    output = run(measurements)
    writeToFile(output)

if __name__ == "__main__":
    args = sys.argv
    if len(args) > 1:
        filename = args[1]
        main(filename)
    else:
        print "Must supply filename"
        sys.exit(1)