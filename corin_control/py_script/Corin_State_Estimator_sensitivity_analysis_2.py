#!/usr/bin/env python
import os
import sys
import csv
from experiment_numbers import *
import matplotlib.pyplot as plt

print os.path.dirname(__file__)

# all experiments
# experiments = [20, 21, 22, 23, 26, 27, 32, 33, 42, 43, 47, 60, 62, 64, 65, 72, 73]
experiments = [22]

if len(sys.argv) > 1:
    par = sys.argv[1]
else:
    par = 'Qbf'

pars = {
    'Qbf': (1, 0, 7),
    'Qbw': (3, 7, 16), #65, 76
    'Th': (7, 16, 26)
}

# 8: max position
# 12: rms position
# 16: ratio position
# 18: max angles
# 22: rms angles
# 26: ratio angles
error_col = 12

exp_list = []
for exp_no in experiments:

    print 'experiment: ' + str(exp_no)

    csv_file = os.path.dirname(__file__) + '/2019_sensitivity_results_'+ str(exp_no) + '_2.csv'
    # csv_file = './2019_sensitivity_results_2/2019_sensitivity_results_'+ str(exp_no) + '_2.csv'

    unordered_list = []
    with open(csv_file, 'rb') as csvfile:
        spamreader = csv.reader(csvfile)
        unordered_list = list(spamreader)[2:] # remove first two rows
        ul = unordered_list

    # ordered list
    # ol = sorted(unordered_list,key=lambda x: x[error_col])
    # ol = unordered_list

    # exp_list.append(ol)
    x = []
    y1 = []
    y2 = []
    col = pars[par][0]
    row_s = pars[par][1]
    row_e = pars[par][2]
    for i in range(row_s,row_e):
        x.append( str (ul[i][col]) )
        y1.append( abs(float(ul[i][12])) )
        y2.append( abs(float(ul[i][22])) )

    print x
    print y1
    plt.figure()

    plt.subplot(2, 1, 1)
    plt.title(str(exp_no))
    # plt.yscale('log')
    if par != "Th":
        plt.xscale('log')
    plt.plot(x, y1)
    plt.ylabel('Position Error')
    axes = plt.gca()
    # axes.set_xlim([xmin,xmax])
    # axes.set_ylim([min(y),max(y)])

    plt.subplot(2, 1, 2)
    # plt.yscale('log')
    if par != "Th":
        plt.xscale('log')
    plt.plot(x, y2)
    plt.ylabel('Angular Error')
    plt.xlabel(par)

plt.show()
