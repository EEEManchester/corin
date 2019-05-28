#!/usr/bin/env python
import os
import sys
import csv
from experiment_numbers import *
import matplotlib.pyplot as plt
import rospkg
import numpy as np

print os.path.dirname(__file__)

# all experiments
experiments = [26, 46, 64, 71, 72, 84]
experiments = ([10, 11, 12, 20, 21, 22, 23, 25, 26, 27, 29, 30, 31, 40, 41, 44,
                45, 46, 48, 49, 50, 70, 71, 72, 80, 81, 82, 84, 85, 86, 88] + [60, 61, 62, 64, 65, 66, 68])
# experiments = [26, 46, 64]

if len(sys.argv) > 1:
    par = sys.argv[1]
else:
    par = 'Qf'

pars = {
    'Qf': (0, 0, 12), # col, row_start, row_end
    'Qbf': (1, 12, 23),
    'Qw': (2, 23, 36),
    'Qbw': (3, 36, 47), #65, 76
    'Qp': (4, 47, 58),
    'Rs': (5, 58, 67),
    'Th': (7, 67, 77),
    'P_r': (8, 77, 88),
    'P_v': (9, 88, 99),
    'P_q': (10, 99, 110),
    'P_p': (11, 110, 121),
    'P_bf': (12, 121, 132),
    'P_bw': (13, 132, 143)
}

# 8: max position
# 12: rms position
# 16: ratio position
# 18: max angles
# 22: rms angles
# 26: ratio angles
# error_col = 12

exp_list = []
pos_stds = []
ang_stds = []
for exp_no in experiments:

    print 'experiment: ' + str(exp_no)

    data_path = rospkg.RosPack().get_path('data')
    # csv_file = os.path.dirname(__file__) + '/2019_sensitivity_results_'+ str(exp_no) + '.csv'
    csv_file = '/2019_sensitivity_results_3/2019_sensitivity_results_'+ str(exp_no) + '_3.csv'

    unordered_list = []
    with open(data_path + csv_file, 'rb') as csvfile:
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

    pos_stds.append(np.std(y1))
    ang_stds.append(np.std(y2))
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


print "Average position error std: ", np.mean(pos_stds)
print "Average angluar error std: ", np.mean(ang_stds)
# plt.figure()
# plt.title("Distributions")
# plt.plot([1]*len(pos_stds) + [2]*len(ang_stds), pos_stds + ang_stds, marker="x", linestyle="None")

plt.show()
