def createList(path):
    f = open(path, "r")
    myList = []
    for line in f:
        myList.append(line.replace("\n", "").split(','))
    return myList

def testList(completeList):
    return completeList[:len(completeList)-60]

def expectedResult(completeList):
    return completeList[len(completeList)-60:]

def main():
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

def run(input):
    # input is a list of lists of locations for every time frame
    # example: [[32,67],[35,67],[36,65],[36,64],[35,63]...]
    # output should be in the same form as input but contain only 60 frames
    output = []
    # enter code here 
    
    # example perfect output from test01.txt: [['1153', '896'], ['1156', '923'], ['1163', '934'], ['1159', '930'], ['1159', '910'], ['1154', '925'], ['1150', '921'], ['1152', '924'], ['1152', '913'], ['1152', '925'], ['1161', '919'], ['1155', '934'], ['1163', '927'], ['1164', '930'], ['1163', '930'], ['1155', '953'], ['1148', '951'], ['1141', '944'], ['1136', '944'], ['1120', '949'], ['1120', '946'], ['1110', '946'], ['1104', '946'], ['1090', '949'], ['1074', '948'], ['1053', '946'], ['1027', '938'], ['1006', '931'], ['993', '926'], ['977', '918'], ['963', '910'], ['938', '897'], ['920', '886'], ['903', '879'], ['880', '865'], ['865', '858'], ['845', '846'], ['821', '834'], ['801', '826'], ['776', '816'], ['751', '810'], ['726', '803'], ['704', '796'], ['679', '790'], ['654', '782'], ['632', '773'], ['613', '761'], ['593', '749'], ['575', '737'], ['560', '728'], ['543', '714'], ['525', '701'], ['507', '688'], ['492', '671'], ['478', '660'], ['465', '645'], ['453', '631'], ['440', '616'], ['426', '594'], ['418', '579']]
    return output
