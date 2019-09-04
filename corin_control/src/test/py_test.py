#!/usr/bin/env python

import numpy as np
import copy
import math
import sys
from termcolor import colored
import argparse
import os
import time

def seq(start, stop, step=1):
	n = int(round((stop - start)/float(step)))
	if n > 1:
		return([start + step*i for i in range(n+1)])
	elif n == 1:
		return([start])
	else:
		return([])

if __name__ == "__main__":

	# print colored("Moving robot to default pose ...", 'green')

	# Set up argument parser
	parser = argparse.ArgumentParser(description='Python test script.')
	# Base pose
	parser.add_argument("-x", "--base_x", type=float, default=0., help="base pose x")
	parser.add_argument("-y", "--base_y", type=float, default=0., help="base pose y")
	parser.add_argument("-z", "--base_z", type=float, default=0., help="base pose z")
	parser.add_argument("-qr", "--base_roll", type=float, default=0., help="base roll")
	parser.add_argument("-qp", "--base_pitch", type=float, default=0., help="base pitch")
	parser.add_argument("-qy", "--base_yaw", type=float, default=0., help="base yaw")
	# Motion
	parser.add_argument("-m", "--motion", type=str, default="normal", help="type of motion")

	args = vars(parser.parse_args())
	print args
	# print("Hi there {}, it's nice to meet you!".format(args["name"]))
	print args["base_x"]
	
	duration = 0.05  # seconds
	freq = 440  # Hz
	for i in range(3):
		os.system('play -nq -t alsa synth {} sine {}'.format(duration, freq))
		time.sleep(0.1)