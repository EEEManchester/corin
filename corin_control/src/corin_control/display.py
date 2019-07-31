#!/usr/bin/env python

""" Functions for printing """

import sys; sys.dont_write_bytecode = True

import numpy as np
from termcolor import colored

def cout(data):
	print data

def cout2(data):
	print np.round(data,2)

def cout3(data):
	print np.round(data,3)

def cout3v(data):
	print np.round(data.flatten(),3)

def cout3p(data):
	print np.round(data[:3,3].flatten(),3)