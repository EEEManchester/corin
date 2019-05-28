#!/usr/bin/env python
import os
import sys
import csv
from experiment_numbers import *

experiments = range(60,69)

# 8: max position
# 12: rms position
# 16: ratio position
# 18: max angles
# 22: rms angles
# 26: ratio angles
error_col = 22
rows = 15

exp_list = []

for exp_no in experiments:

    print 'experiment: ' + str(exp_no)

    csv_file = './2019_optimisation_results_2/2019_optimisation_results_'+ str(exp_no) + '_2.csv'

    unordered_list = []
    with open(csv_file, 'rb') as csvfile:
        spamreader = csv.reader(csvfile)
        unordered_list = list(spamreader)[2:] # remove first two rows

    # ordered list
    ol = sorted(unordered_list,key=lambda x: x[error_col])[0:rows]
    # ol = unordered_list

    for i in ol:
        print i[0:8] + [i[error_col]] + [i[16]]
