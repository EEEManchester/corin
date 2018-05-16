#!/usr/bin/env python
# stance selection for corin

import sys; sys.dont_write_bytecode = True

import numpy as np
from matrix_transforms import *

## ================================================================ ##
##                       Stance Selection 	 						##
## ================================================================ ##

def initial_stance(STANCE_WIDTH, BODY_HEIGHT, type="flat", TETA_F=20, TETA_R=-20):
    LEG_STANCE = {}

    # Flat ground stance - original
    if (type=="flat"):
        print 'Flat selected'
        LEG_STANCE[0] = np.array([ STANCE_WIDTH*np.cos(TETA_F*np.pi/180), STANCE_WIDTH*np.sin(TETA_F*np.pi/180), -BODY_HEIGHT ])
        LEG_STANCE[1] = np.array([ STANCE_WIDTH, 0, -BODY_HEIGHT])
        LEG_STANCE[2] = np.array([ STANCE_WIDTH*np.cos(TETA_R*np.pi/180), STANCE_WIDTH*np.sin(TETA_R*np.pi/180), -BODY_HEIGHT ])

        LEG_STANCE[3] = np.array([ STANCE_WIDTH*np.cos(TETA_F*np.pi/180), STANCE_WIDTH*np.sin(-TETA_F*np.pi/180), -BODY_HEIGHT ])
        LEG_STANCE[4] = np.array([ STANCE_WIDTH, 0, -BODY_HEIGHT])
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
        LEG_STANCE[4] = np.array([STANCE_WIDTH, 0, -BODY_HEIGHT])   # np.array([STANCE_WIDTH-0.1, 0, -BODY_HEIGHT])
        LEG_STANCE[5] = np.array([ xrr, yrr, -BODY_HEIGHT ])

        return LEG_STANCE

    ## Sideways walking stance
    elif (type=="sideways"):
        print 'Sideways selected'
        ## Stance for left & right side

        ## 30 deg
        V_RS_WIDTH =  0.284;    V_LS_WIDTH = 0.234
        V_RS_HEIGH = -0.04;     V_LS_HEIGH = 0.061

        ## 60 deg
        # V_RS_WIDTH =  0.288;    V_LS_WIDTH = 0.274
        # V_RS_HEIGH = -0.019;    V_LS_HEIGH = 0.044

        # ## 90 deg
        # V_RS_WIDTH = 0.325;     V_LS_WIDTH = 0.18
        # V_RS_HEIGH = 0.097;     V_LS_HEIGH = -0.1

        phi = 40. - TETA_F
        V_RS_H  = V_RS_WIDTH/np.cos(phi*np.pi/180)
        V_LS_H  = V_LS_WIDTH/np.cos(phi*np.pi/180)

        LEG_STANCE[0] = np.array([ V_LS_H*np.cos( TETA_F*np.pi/180), V_LS_H*np.sin( TETA_F*np.pi/180), V_LS_HEIGH ])
        LEG_STANCE[1] = np.array([ V_LS_WIDTH, 0, V_LS_HEIGH ])
        LEG_STANCE[2] = np.array([ V_LS_H*np.cos(-TETA_F*np.pi/180), V_LS_H*np.sin(-TETA_F*np.pi/180), V_LS_HEIGH ])

        LEG_STANCE[3] = np.array([ V_RS_H*np.cos(-TETA_F*np.pi/180), V_RS_H*np.sin(-TETA_F*np.pi/180), V_RS_HEIGH ])
        LEG_STANCE[4] = np.array([ V_RS_WIDTH, 0, V_RS_HEIGH])
        LEG_STANCE[5] = np.array([ V_RS_H*np.cos( TETA_F*np.pi/180), V_RS_H*np.sin( TETA_F*np.pi/180), V_RS_HEIGH ])
        

        # LEG_STANCE[0] = np.array([ V_LS_WIDTH*np.cos(TETA_F*np.pi/180), V_LS_WIDTH*np.sin(TETA_F*np.pi/180), V_LS_HEIGH ])
        # LEG_STANCE[1] = np.array([ V_LS_WIDTH*np.cos((40-TETA_F)*np.pi/180), 0, V_LS_HEIGH ])
        # LEG_STANCE[2] = np.array([ V_LS_WIDTH*np.cos(TETA_R*np.pi/180), V_LS_WIDTH*np.sin(TETA_R*np.pi/180), V_LS_HEIGH ])

        # LEG_STANCE[3] = np.array([ V_RS_WIDTH*np.cos(TETA_F*np.pi/180), V_RS_WIDTH*np.sin(-TETA_F*np.pi/180), V_RS_HEIGH ])
        # LEG_STANCE[4] = np.array([ V_RS_WIDTH*np.cos((40-TETA_F)*np.pi/180), 0, V_RS_HEIGH])
        # LEG_STANCE[5] = np.array([ V_RS_WIDTH*np.cos(TETA_R*np.pi/180), V_RS_WIDTH*np.sin(-TETA_R*np.pi/180), V_RS_HEIGH ])

        return LEG_STANCE

## ================================================================ ##
##                             Leg Pitch                            ##
## ================================================================ ##

def set_step_stroke(tx_bXl, rot_bXl, leg_stance, d_clear, stroke_default=0.1):
    """ Sets the size of step stroke """
    
    p_base_X_foot_0 = np.array([tx_bXl[0][0], tx_bXl[0][1], 0.]) + mX(rot_Z(np.deg2rad(rot_bXl[0])),leg_stance[0])
    p_base_X_foot_1 = np.array([tx_bXl[1][0], tx_bXl[1][1], 0.]) + mX(rot_Z(np.deg2rad(rot_bXl[1])),leg_stance[1])

    # Compute pitch
    nrp_pitch = p_base_X_foot_0 - p_base_X_foot_1
    x_pitch = (nrp_pitch[0] - d_clear)/2.

    # Set stroke
    sstroke = 2*x_pitch if (stroke_default/2 > x_pitch) else stroke_default
    return sstroke
