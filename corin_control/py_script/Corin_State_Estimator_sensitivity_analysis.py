#!/usr/bin/env python
import os
import sys
import csv
from experiment_numbers import *
import matplotlib.pyplot as plt

print os.path.dirname(__file__)

# all experiments
# experiments = [10, 11, 22, 26, 30, 41, 42, 46, 50, 71, 72, 80, 81, 84, 87] + range(60, 69)
# one of each
experiments = [26, 46, 64, 71, 72, 84]
# only rotations
# experiments = [10, 11] + range(60, 69)
# experiments = [60]

if len(sys.argv) > 1:
    par = sys.argv[1]
else:
    par = 'Qf'

pars = {
    'Qf': (0, 0, 11), # col, row_start, row_end
    'Qbf': (1, 11, 22),
    'Qw': (2, 22, 33),
    'Qbw': (3, 33, 44), #65, 76
    'Qp': (4, 44, 55),
    'Rs': (5, 55, 64),
    'Th': (7, 65, 75)
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

    # csv_file = os.path.dirname(__file__) + '/2019_sensitivity_results_'+ str(exp_no) + '.csv'
    csv_file = './2019_sensitivity_results/2019_sensitivity_results_'+ str(exp_no) + '.csv'

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
    y = []
    col = pars[par][0]
    row_s = pars[par][1]
    row_e = pars[par][2]
    for i in range(row_s,row_e):
        x.append( str (ul[i][col]) )
        y.append( abs(float(ul[i][error_col])) )

    # print x
    # print y
    plt.figure()
    plt.plot(x, y, label="Estimate-x")
    axes = plt.gca()
    # axes.set_xlim([xmin,xmax])
    # axes.set_ylim([min(y),max(y)])
    # plt.yscale('log')
    if par != "Th":
        plt.xscale('log')
    plt.legend()
    plt.title(str(exp_no))
    plt.xlabel(par)
    plt.ylabel('Error')

plt.show()
