#!/usr/bin/env python
# stance selection for corin

import sys; sys.dont_write_bytecode = True

from constant import *
import numpy as np

## ================================================================ ##
##                       Stance Selection 	 						##
## ================================================================ ##

def initial_stance(STANCE_WIDTH, BODY_HEIGHT, type="flat", TETA_F=20, TETA_R=-20):
    LEG_STANCE = {}

    # Flat ground stance - original
    if (type=="flat"):
        print 'Flat original selected'
        LEG_STANCE[0] = np.array([ STANCE_WIDTH*np.cos(TETA_F*np.pi/180), STANCE_WIDTH*np.sin(TETA_F*np.pi/180), -BODY_HEIGHT ])
        LEG_STANCE[1] = np.array([ STANCE_WIDTH, 0, -BODY_HEIGHT])
        LEG_STANCE[2] = np.array([ STANCE_WIDTH*np.cos(TETA_R*np.pi/180), STANCE_WIDTH*np.sin(TETA_R*np.pi/180), -BODY_HEIGHT ])

        LEG_STANCE[3] = np.array([ STANCE_WIDTH*np.cos(TETA_F*np.pi/180), STANCE_WIDTH*np.sin(-TETA_F*np.pi/180), -BODY_HEIGHT ])
        LEG_STANCE[4] = np.array([STANCE_WIDTH, 0, -BODY_HEIGHT])
        LEG_STANCE[5] = np.array([ STANCE_WIDTH*np.cos(TETA_R*np.pi/180), STANCE_WIDTH*np.sin(-TETA_R*np.pi/180), -BODY_HEIGHT ])

        return LEG_STANCE

    ## Flat legs aligned walking stance
    elif (type=="chimney"):
        print 'Chimney selected'
        x02 = STANCE_WIDTH;

        ## front left
        q = -40*np.pi/180
        y02 = -STANCE_WIDTH*np.tan((40-TETA_F)*np.pi/180)
        xfl = x02*np.cos(q) + y02*np.sin(q)
        yfl = y02*np.cos(q) - x02*np.sin(q)

        ## rear left
        q = 40*np.pi/180
        y02 = STANCE_WIDTH*np.tan((40+TETA_R)*np.pi/180)
        xrl = x02*np.cos(q) + y02*np.sin(q)
        yrl = y02*np.cos(q) - x02*np.sin(q)

        ## front right
        q = 40*np.pi/180
        y02 = STANCE_WIDTH*np.tan((40-TETA_F)*np.pi/180)
        xfr = x02*np.cos(q) + y02*np.sin(q)
        yfr = y02*np.cos(q) - x02*np.sin(q)
        # print 'xy: ', xfr, yfr

        ## rear right
        q = -40*np.pi/180
        y02 = -STANCE_WIDTH*np.tan((40+TETA_R)*np.pi/180)
        xrr = x02*np.cos(q) + y02*np.sin(q)
        yrr = y02*np.cos(q) - x02*np.sin(q)
        # print 'xy: ', xrr, yrr

        LEG_STANCE[0] = np.array([ xfl ,yfl , -BODY_HEIGHT ])
        LEG_STANCE[1] = np.array([ STANCE_WIDTH, 0, -BODY_HEIGHT])
        LEG_STANCE[2] = np.array([ xrl ,yrl, -BODY_HEIGHT ])

        LEG_STANCE[3] = np.array([ xfr, yfr, -BODY_HEIGHT ])
        LEG_STANCE[4] = np.array([STANCE_WIDTH, 0, -BODY_HEIGHT])
        LEG_STANCE[5] = np.array([ xrr, yrr, -BODY_HEIGHT ])

        return LEG_STANCE

    ## Sideways walking stance
    elif (type=="sideways"):
        print 'Sideways selected'
        ## robot at 45deg according to matlab sim
        V_RS_WIDTH = 0.2407
        V_RS_HEIGH = 0.0576

        V_LS_WIDTH = 0.215
        V_LS_HEIGH = -0.081

        LEG_STANCE[0] = np.array([ V_LS_WIDTH*np.cos(TETA_F*np.pi/180), V_LS_WIDTH*np.sin(TETA_F*np.pi/180), V_LS_HEIGH ])
        LEG_STANCE[1] = np.array([ V_LS_WIDTH*np.cos((40-TETA_F)*np.pi/180), 0, V_LS_HEIGH ])
        LEG_STANCE[2] = np.array([ V_LS_WIDTH*np.cos(TETA_R*np.pi/180), V_LS_WIDTH*np.sin(TETA_R*np.pi/180), V_LS_HEIGH ])

        LEG_STANCE[3] = np.array([ V_RS_WIDTH*np.cos(TETA_F*np.pi/180), V_RS_WIDTH*np.sin(-TETA_F*np.pi/180), V_RS_HEIGH ])
        LEG_STANCE[4] = np.array([ V_RS_WIDTH*np.cos((40-TETA_F)*np.pi/180), 0, V_RS_HEIGH])
        LEG_STANCE[5] = np.array([ V_RS_WIDTH*np.cos(TETA_R*np.pi/180), V_RS_WIDTH*np.sin(-TETA_R*np.pi/180), V_RS_HEIGH ])

        return LEG_STANCE
