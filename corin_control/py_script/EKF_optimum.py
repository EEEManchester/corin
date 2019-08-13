#!/usr/bin/env python
import os
import sys
import csv
from experiment_numbers import *
import rospkg


experiments = [10, 22, 26, 41, 42, 46, 50, 65, 71, 72, 80, 84]
top_th = 100 # top rows to look at

# 8: max position
# 12: rms position
# 16: ratio position
# 18: max angles
# 22: rms angles
# 26: ratio angles
error_col = 12
max_list = ['']*top_th

exp_list = []
for exp_no in experiments:

    print 'experiment: ' + str(exp_no)

    data_path = rospkg.RosPack().get_path('data')
    csv_file = data_path + '/2019_optimisation_results/2019_optimisation_results_'+ str(exp_no) + '.csv'

    unordered_list = []
    with open(csv_file, 'rb') as csvfile:
        spamreader = csv.reader(csvfile)
        unordered_list = list(spamreader)[2:] # remove first two rows

    # ordered list
    ol = sorted(unordered_list,key=lambda x: x[error_col])
    # ol = unordered_list

    exp_list.append(ol)

for i in range(top_th):
    max_list [i] = i
    # print "Searching for the parameters below:"
    arr1 = exp_list[0][i][0:8]

    for j in range(1, len(experiments)):
        max_rows = len(exp_list[j])

        for k in range(max_rows):
            arr2 = exp_list[j][k][0:8]
            if arr1 == arr2:
                if k > max_list[i]:
                    max_list[i] = k
                # exit()
                break


unordered_max_list = []
for i in range(len(max_list)):
    unordered_max_list.append([max_list[i]] + exp_list[0][i][0:8] + [exp_list[0][i][error_col]])

ordered_max_list = sorted(unordered_max_list,key=lambda x: x[0])[0:10]
for i in ordered_max_list:
    print i
