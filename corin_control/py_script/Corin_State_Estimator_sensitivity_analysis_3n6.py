#!/usr/bin/env python
import os
import sys
import csv
from experiment_numbers import *
import matplotlib.pyplot as plt
import rospkg
print os.path.dirname(__file__)

# all experiments
experiments = [26, 46, 64, 71, 72, 84]
experiments = ([10, 11, 12, 20, 21, 22, 23, 25, 26, 27, 40, 41,
                46, 70, 71, 72, 81, 84] + [61, 64])


if len(sys.argv) > 1:
    par = sys.argv[1]
else:
    par = 'Qf'

pars3 = {
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


pars6 = {
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

    csv_file = '/2019_sensitivity_results_3/2019_sensitivity_results_'+ str(exp_no) + '_3.csv'

    unordered_list = []
    with open(data_path + csv_file, 'rb') as csvfile:
        spamreader = csv.reader(csvfile)
        unordered_list = list(spamreader)[2:] # remove first two rows
        ul3 = unordered_list

    csv_file = '/2019_sensitivity_results_6/2019_sensitivity_results_'+ str(exp_no) + '_6.csv'

    unordered_list = []
    with open(data_path + csv_file, 'rb') as csvfile:
        spamreader = csv.reader(csvfile)
        unordered_list = list(spamreader)[2:] # remove first two rows
        ul6 = unordered_list

    x3 = []
    y3a = []
    y3b = []
    col = pars3[par][0]
    row_s = pars3[par][1]
    row_e = pars3[par][2]
    for i in range(row_s,row_e):
        x3.append( str (ul3[i][col]) )
        y3a.append( abs(float(ul3[i][18])) )
        y3b.append( abs(float(ul3[i][28])) )

    x6 = []
    y6a = []
    y6b = []
    col = pars6[par][0]
    row_s = pars6[par][1]
    row_e = pars6[par][2]
    for i in range(row_s,row_e):
        x6.append( str (ul6[i][col]) )
        y6a.append( abs(float(ul6[i][18])) )
        y6b.append( abs(float(ul6[i][28])) )

    # print x
    # print y
    plt.figure()

    plt.subplot(2, 1, 1)
    plt.title(str(exp_no))
    # plt.yscale('log')
    if par != "Th":
        plt.xscale('log')
    plt.plot(x3, y3a, label="init")
    plt.plot(x6, y6a, label="none")
    plt.legend()
    plt.ylabel('Position Error')
    axes = plt.gca()
    # axes.set_xlim([xmin,xmax])
    axes.set_ylim([0,max(y3a+y6a)])

    plt.subplot(2, 1, 2)
    # plt.yscale('log')
    axes = plt.gca()
    axes.set_ylim([0,max(y3b+y6b)])
    if par != "Th":
        plt.xscale('log')
    plt.plot(x3, y3b, label="init")
    plt.plot(x6, y6b, label="none")
    plt.legend()
    plt.ylabel('Angular Error')
    plt.xlabel(par)

plt.show()
