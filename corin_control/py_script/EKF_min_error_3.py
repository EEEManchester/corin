#!/usr/bin/env python
import os
import sys
import csv
from experiment_numbers import *

# experiments = range(60,69)
# experiments = [11, 42, 64, 23, 29, 83, 60, 47, 10, 68, 76, 74, 87, 43, 65, 30, 49, 61, 25, 84, 80, 88, 77, 70]
experiments = [87, 30, 64, 26, 84, 61, 81, 22, 50, 72, 11, 46, 71, 80, 10, 67, 42]

# 9: max position
# 13: rms position
# 17: ratio position
# 19: max angles
# 23: rms angles
# 27: ratio angles
error_col = 13
rows = 15

exp_list = []

for exp_no in experiments:

    print 'experiment: ' + str(exp_no)

    csv_file = './2019_optimisation_results_3/2019_optimisation_results_'+ str(exp_no) + '_3.csv'

    unordered_list = []
    with open(csv_file, 'rb') as csvfile:
        spamreader = csv.reader(csvfile)
        unordered_list = list(spamreader)[2:] # remove first two rows

    # ordered list
    ol = sorted(unordered_list,key=lambda x: x[error_col])[0:rows]
    # ol = unordered_list

    for i in ol:
        print i[4:6] + [i[error_col]] + [i[17]]
