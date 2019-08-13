#!/usr/bin/env python
import os
import sys
import csv
from experiment_numbers import *
import rospkg

experiments = [10, 22, 26, 41, 42, 46, 50, 65, 71, 72, 80, 84]

# 8: max position
# 12: rms position
# 16: ratio position
# 18: max angles
# 22: rms angles
# 26: ratio angles
error_col = 12
rows = 15

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
    ol = sorted(unordered_list,key=lambda x: x[error_col])[0:rows]
    # ol = unordered_list

    for i in ol:
        print i[0:8] + [i[error_col]] + [i[16]]
