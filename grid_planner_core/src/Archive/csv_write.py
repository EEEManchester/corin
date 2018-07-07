#!/usr/bin/env python

import csv
import numpy as np

a = np.array([1,2])
b = np.array([1,2])

c = np.hstack((a,b,a,b))
print c
# with open('eggs.csv', 'wb') as csvfile:
#     spamwriter = csv.writer(csvfile, delimiter='\t')#, quotechar='|', quoting=csv.QUOTE_MINIMAL)
#     spamwriter.writerow(['Saaapaddm'] * 5 + ['Baked Beans'])
#     spamwriter.writerow(['Spam', 'Lovefdly Spam', 'Wonderful Spam'])