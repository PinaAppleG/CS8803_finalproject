import os, sys

NUM_INPUTS = 10

if len(sys.argv) != 2:
    print >> sys.stderr, "ERROR Usage: %s finalproject" % sys.argv[0]
    print >> sys.stderr, "('finalproject' is the directory that contains final project files)"
    sys.exit(1)

directory = sys.argv[1]

def error(l1, l2):
    return sum((c - a)**2 + (d - b)**2 for ((a, b), (c, d)) in zip(l1, l2))**0.5

def convert_line(line):
    x, y = line.split(',')
    return int(x.strip()), int(y.strip())

inputs = [directory + "/inputs/test%02d.txt" % i for i in range(1, NUM_INPUTS+1)]
actuals = ["actual/%02d.txt" % i for i in range(1, NUM_INPUTS+1)]
tests = []
for i, filename in enumerate(inputs):
    os.system('python %s/finalproject.py %s' % (directory, filename))
    tests.append(([convert_line(line) for line in open('prediction.txt', 'r').readlines()],
                  [convert_line(line) for line in open(actuals[i], 'r').readlines()]))

errors = sorted([error(l1, l2) for (l1, l2) in tests])
print sum(errors[1:-1]) / (NUM_INPUTS - 2)
