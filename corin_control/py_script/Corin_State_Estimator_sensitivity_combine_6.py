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
experiments = ([10, 11, 12, 20, 21, 22, 23, 24, 25, 26, 27, 28, 40, 41, 42, 43, 44,
                46, 70, 71, 72, 81, 84, 87] + [61, 64, 67])

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


exp_list = []
position_error = []
angles_error = []
first_i = True
for exp_no in experiments:

    print 'experiment: ' + str(exp_no)

    data_path = rospkg.RosPack().get_path('data')

    csv_file = data_path + '/2019_sensitivity_results_6/2019_sensitivity_results_'+ str(exp_no) + '_6.csv'

    unordered_list = []
    with open(csv_file, 'rb') as csvfile:
        spamreader = csv.reader(csvfile)
        unordered_list = list(spamreader)[2:] # remove first two rows
        ul = unordered_list

    # ordered list
    # ol = sorted(unordered_list,key=lambda x: x[error_col])
    # ol = unordered_list

    x = ['']
    y1 = [exp_no]
    y2 = [exp_no]
    col = pars[par][0]
    row_s = pars[par][1]
    row_e = pars[par][2]
    for i in range(row_s,row_e):
        x.insert(1, str (ul[i][col]) )
        y1.insert(1, abs(float(ul[i][18])) )
        y2.insert(1, abs(float(ul[i][28])) )

    if first_i:
        position_error.append(x)
        angles_error.append(x)
        first_i = False
    position_error.append(y1)
    angles_error.append(y2)

    # print(zip(*position_error))
    # print(zip(*angles_error))

csv_position = './2019_sensitivity_results_6/2019_sensitivity_results_'+ par + '_position_error_6.csv'
csv_angles = './2019_sensitivity_results_6/2019_sensitivity_results_'+ par + '_angles_error_6.csv'

with open(csv_position,'wb') as myfile:
  wr = csv.writer(myfile) #, quoting=csv.QUOTE_ALL)
  wr.writerows(zip(*position_error))

with open(csv_angles,'wb') as myfile:
  wr = csv.writer(myfile) #, quoting=csv.QUOTE_ALL)
  wr.writerows(zip(*angles_error))
