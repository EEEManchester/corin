#!/usr/bin/env python
import os
import sys
import csv
from experiment_numbers import *
import matplotlib.pyplot as plt

print os.path.dirname(__file__)

# all experiments
experiments = [26, 46, 64, 71, 72, 84]
experiments = ([10, 11, 12, 20, 21, 22, 23, 25, 26, 27, 29, 30, 31, 40, 41, 44,
                45, 46, 48, 49, 50, 70, 71, 72, 80, 81, 82, 84, 85, 86, 88] + [60, 61, 62, 64, 65, 66, 68])

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


exp_list = []
position_error = []
angles_error = []
first_i = True
for exp_no in experiments:

    print 'experiment: ' + str(exp_no)


    # csv_file = os.path.dirname(__file__) + '/2019_sensitivity_results_'+ str(exp_no) + '.csv'
    csv_file = './2019_sensitivity_results_3/2019_sensitivity_results_'+ str(exp_no) + '_3.csv'

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

csv_position = './2019_sensitivity_results_3/2019_sensitivity_results_'+ par + '_position_error_3.csv'
csv_angles = './2019_sensitivity_results_3/2019_sensitivity_results_'+ par + '_angles_error_3.csv'

with open(csv_position,'wb') as myfile:
  wr = csv.writer(myfile) #, quoting=csv.QUOTE_ALL)
  wr.writerows(zip(*position_error))

with open(csv_angles,'wb') as myfile:
  wr = csv.writer(myfile) #, quoting=csv.QUOTE_ALL)
  wr.writerows(zip(*angles_error))
