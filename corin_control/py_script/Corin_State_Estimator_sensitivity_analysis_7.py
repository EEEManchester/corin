#!/usr/bin/env python
import os
import sys
import csv
from experiment_numbers import *
import matplotlib.pyplot as plt
import rospkg

# all experiments
experiments = [26, 46, 64, 71, 72, 84]
experiments = ([10, 11, 12, 20, 21, 22, 23, 24, 25, 26, 27, 28, 40, 41, 42, 43, 44,
                46, 70, 71, 72, 81, 84, 87] + [61, 64, 67])
# experiments = [72]

if len(sys.argv) > 1:
    par = sys.argv[1]
else:
    par = 'Qf'

pars = {
    'Qf': (0, 0, 10), # col, row_start, row_end
    'Qbf': (1, 10, 18),
    'Qw': (2, 18, 29),
    'Qbw': (3, 29, 39), #65, 76
    'Qp': (4, 39, 49),
    'Rs': (5, 49, 58),
    'P_r': (8, 58, 63),
    'P_v': (9, 63, 74),
    'P_q': (10, 74, 85),
    'P_bf': (12, 85, 96),
    'P_bw': (13, 96, 109)
}

# 8: max position
# 12: rms position
# 16: ratio position
# 18: max angles
# 22: rms angles
# 26: ratio angles
# error_col = 12

exp_list = []
for exp_no in experiments:

    print 'experiment: ' + str(exp_no)

    data_path = rospkg.RosPack().get_path('data')
    csv_file = data_path + '/2019_sensitivity_results_7/2019_sensitivity_results_'+ str(exp_no) + '_7.csv'

    unordered_list = []
    with open(csv_file, 'rb') as csvfile:
        spamreader = csv.reader(csvfile)
        unordered_list = list(spamreader)[2:] # remove first two rows
        ul = unordered_list

    # ordered list
    # ol = sorted(unordered_list,key=lambda x: x[error_col])
    # ol = unordered_list

    x = []
    y1 = []
    y2 = []
    col = pars[par][0]
    row_s = pars[par][1]
    row_e = pars[par][2]
    for i in range(row_s,row_e):
        x.append( str (ul[i][col]) )
        y1.append( abs(float(ul[i][18])) )
        y2.append( abs(float(ul[i][28])) )

    # print x
    # print y
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
    axes.set_ylim([0,max(y1)])

    plt.subplot(2, 1, 2)
    # plt.yscale('log')
    axes = plt.gca()
    axes.set_ylim([0,max(y2)])
    if par != "Th":
        plt.xscale('log')
    plt.plot(x, y2)
    plt.ylabel('Angular Error')
    plt.xlabel(par)

plt.show()
