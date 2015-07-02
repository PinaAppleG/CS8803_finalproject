def createList(path):
    f = open(path, "r")
    myList = []
    for line in f:
        myList.append(line.replace("\n", "").split(','))
    return myList
