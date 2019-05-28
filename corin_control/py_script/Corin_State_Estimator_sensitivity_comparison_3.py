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

# if len(sys.argv) > 1:
#     par = sys.argv[1]
# else:
#     par = 'Qf'

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

all_x = []
all_pos_stds = []
all_ang_stds = []
pars_list = ['Qf', 'Qbf', 'Qw', 'Qbw', 'Qp', 'Rs', 'Th', 'P_r', 'P_v', 'P_q', 'P_p', 'P_bf', 'P_bw']
for k, par in enumerate(pars_list):
    print par

    exp_list = []
    exp_pos_stds = []
    exp_ang_stds = []
    for exp_no in experiments:

        # print 'experiment: ' + str(exp_no)

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

        # Ignore transient
        if par == "P_q":
            y1 = y1[3:]
            y2 = y2[4:]
        exp_pos_stds.append(np.std(y1))
        exp_ang_stds.append(np.std(y2))
        # exp_pos_stds.append(max(y1)- min(y1))
        # exp_ang_stds.append(max(y2) - min(y2))

    all_pos_stds += exp_pos_stds
    all_ang_stds += exp_ang_stds
    all_x += [k] * len(exp_pos_stds)

    print "Average position error std: ", np.mean(exp_pos_stds)
    print "Average angluar error std: ", np.mean(exp_ang_stds)


plt.figure()
plt.title("Distributions")
plt.plot(all_x, all_pos_stds, marker="x", linestyle="None")

# np.savetxt(data_path + "/2019_sensitivity_comparison_3.csv", zip(all_x, all_pos_stds, all_ang_stds), delimiter=",")

plt.show()
