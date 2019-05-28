experiment = ['']*100

# August experiments (no offset)
experiment[0] = '/experiments_2018_Aug/corin_walking_0.bag'
experiment[1] = '/experiments_2018_Aug/corin_walking_1.bag'
experiment[2] = '/experiments_2018_Aug/corin_walking_2.bag'
experiment[3] = '/experiments_2018_Aug/corin_walking_3.bag'
experiment[4] = '/experiments_2018_Aug/corin_walking_4.bag'
experiment[5] = '/experiments_2018_Aug/corin_walking_5.bag'
experiment[6] = '/experiments_2018_Aug/corin_walking_6.bag'

# bodypose: 3
experiment[10] = '/experiments_2018_Dec/corin_bodypose_1.bag'
experiment[11] = '/experiments_2018_Dec/corin_bodypose_2.bag' # messages: 138645
experiment[12] = '/experiments_2018_Dec/corin_bodypose_3.bag'

# front/back movements: 12 + 2 (extra)
experiment[20] = '/experiments_2018_Dec/corin_front_wave_0.1.bag'
experiment[21] = '/experiments_2018_Dec/corin_front_wave_0.08.bag'
experiment[22] = '/experiments_2018_Dec/corin_front_wave_0.04.bag'
experiment[23] = '/experiments_2018_Dec/corin_front_wave_0.02.bag'
experiment[24] = '/experiments_2018_Dec/corin_front_tetra_0.1.bag'
experiment[25] = '/experiments_2018_Dec/corin_front_tetra_0.08.bag'
experiment[26] = '/experiments_2018_Dec/corin_front_tetra_0.04.bag'
experiment[27] = '/experiments_2018_Dec/corin_front_tetra_0.02.bag'
experiment[28] = '/experiments_2018_Dec/corin_front_tripod_0.1.bag'
experiment[29] = '/experiments_2018_Dec/corin_front_tripod_0.08.bag'
experiment[30] = '/experiments_2018_Dec/corin_front_tripod_0.04.bag'
experiment[31] = '/experiments_2018_Dec/corin_front_tripod_0.02.bag'
#
experiment[32] = '/experiments_2018_Dec/corin_front_tripod_0.01.bag'
experiment[33] = '/experiments_2018_Dec/corin_front_tetra_0.02b.bag' # 27 repeat

# side to side movements: 12
experiment[40] = '/experiments_2018_Dec/corin_side_wave_0.1.bag'
experiment[41] = '/experiments_2018_Dec/corin_side_wave_0.08.bag'
experiment[42] = '/experiments_2018_Dec/corin_side_wave_0.04.bag'
experiment[43] = '/experiments_2018_Dec/corin_side_wave_0.02.bag' # no offset
experiment[44] = '/experiments_2018_Dec/corin_side_tetra_0.1.bag'
experiment[45] = '/experiments_2018_Dec/corin_side_tetra_0.08.bag'
experiment[46] = '/experiments_2018_Dec/corin_side_tetra_0.04.bag' # slight error with foot 4 at start
experiment[47] = '/experiments_2018_Dec/corin_side_tetra_0.02.bag'
experiment[48] = '/experiments_2018_Dec/corin_side_tripod_0.1.bag'
experiment[49] = '/experiments_2018_Dec/corin_side_tripod_0.08.bag'
experiment[50] = '/experiments_2018_Dec/corin_side_tripod_0.04.bag'
experiment[51] = '/experiments_2018_Dec/corin_side_tripod_0.02.bag'

# rotate: 9
experiment[60] = '/experiments_2018_Dec/corin_rotate_wave_0.08.bag' # no offset (Fri afternoon)
experiment[61] = '/experiments_2018_Dec/corin_rotate_wave_0.04.bag' # no offset
experiment[62] = '/experiments_2018_Dec/corin_rotate_wave_0.02.bag' # no offset
experiment[63] = '/experiments_2018_Dec/corin_rotate_tetra_0.08.bag' # no offset
experiment[64] = '/experiments_2018_Dec/corin_rotate_tetra_0.04.bag' # no offset
experiment[65] = '/experiments_2018_Dec/corin_rotate_tetra_0.02.bag' # no offset
experiment[66] = '/experiments_2018_Dec/corin_rotate_tripod_0.08.bag'
experiment[67] = '/experiments_2018_Dec/corin_rotate_tripod_0.04.bag'
experiment[68] = '/experiments_2018_Dec/corin_rotate_tripod_0.02.bag'

# wall walking: 8
experiment[70] = '/experiments_2018_Dec/corin_wall_wave_0.1.bag'
experiment[71] = '/experiments_2018_Dec/corin_wall_wave_0.08.bag'
experiment[72] = '/experiments_2018_Dec/corin_wall_wave_0.04.bag' # no offset (Wed morning)
experiment[73] = '/experiments_2018_Dec/corin_wall_wave_0.02.bag' # no offset
experiment[74] = '/experiments_2018_Dec/corin_wall_wave_0.01.bag' # no offset
experiment[75] = '/experiments_2018_Dec/corin_wall_tetra_0.1.bag'
experiment[76] = '/experiments_2018_Dec/corin_wall_tetra_0.08.bag'
experiment[77] = '/experiments_2018_Dec/corin_wall_tetra_0.04.bag'

# front on carpet: 9
experiment[80] = '/experiments_2018_Dec/corin_carpet_front_wave_0.08.bag'
experiment[81] = '/experiments_2018_Dec/corin_carpet_front_wave_0.04.bag'
experiment[82] = '/experiments_2018_Dec/corin_carpet_front_wave_0.02.bag'
experiment[83] = '/experiments_2018_Dec/corin_carpet_front_tetra_0.08.bag'
experiment[84] = '/experiments_2018_Dec/corin_carpet_front_tetra_0.04.bag'
experiment[85] = '/experiments_2018_Dec/corin_carpet_front_tetra_0.02.bag'
experiment[86] = '/experiments_2018_Dec/corin_carpet_front_tripod_0.08.bag'
experiment[87] = '/experiments_2018_Dec/corin_carpet_front_tripod_0.04.bag'
experiment[88] = '/experiments_2018_Dec/corin_carpet_front_tripod_0.02.bag'


no_offset_list = range(0,7) + [43] + range(60,66) + range(72,75)

max_messages = ['']*100
max_messages[10] =  120000
max_messages[11] =  125000
max_messages[12] =  120000

max_messages[20] =  900000 # broken return
max_messages[21] =  920000 # returns faster to 13500000
max_messages[22] = 1700000 # return trip stopped
max_messages[23] = 2100000 # no return
max_messages[24] =  230000 # return to 500000
max_messages[25] =  245000 # returns to 500000
max_messages[26] =  450000 # returns back faster to  710000
max_messages[27] =  850000 # returns back faster to 1010000
max_messages[28] =  220000 # returns to 500000
max_messages[29] =  225000 # returns to 500000
max_messages[30] =  260000 # returns to 520000
max_messages[31] =  440000 # returns faster to 710000
max_messages[32] =  790000 # returns faster to 1100000
max_messages[33] =  820000 # returns faster to 1070000
#side
max_messages[40] =  655000 # returns to 1250000
max_messages[41] =  765000 # returns at 1630000
max_messages[42] = 1580000 # returns at 3200000
max_messages[43] = 2732000 # returns to 5600000
max_messages[44] =  230000 # returns to 460000
max_messages[45] =  245000 # returns to 500000
max_messages[46] =  440000 # returns to 900000
max_messages[47] =  810000 # returns slower to 2080000
max_messages[48] =  220000 # returns to 450000
max_messages[49] =  220000 # returns to 500000
max_messages[50] =  245000 # returns to 500000
max_messages[51] =  435000 # returns to 900000
# rotate
max_messages[60] = 1000000
max_messages[61] =  500000
max_messages[62] = 2000000
max_messages[63] =  500000
max_messages[64] =  600000
max_messages[65] = 1000000
max_messages[66] =  450000
max_messages[67] =  480000
max_messages[68] =  700000
# wall
max_messages[70] =  400000
max_messages[71] =  400000
max_messages[72] =  650000
max_messages[73] = 1220000
max_messages[74] =  700000
max_messages[75] =  300000
max_messages[76] =  300000
max_messages[77] =  300000
# carpet
max_messages[80] =  920000
max_messages[81] = 1700000
max_messages[82] = 2500000
max_messages[83] =  250000
max_messages[84] =  450000
max_messages[85] =  850000
max_messages[86] =  220000
max_messages[87] =  250000
max_messages[88] =  450000

# "Optimal" EKF parameters obtained using PSO
# [log_10(qf),log_10(qbf),log_10(qw),log_10(qbw),log_10(qp),log_10(rs),Th]
optimal = [0]*100
optimal[10] = [-1.00734,-6.61554,-8.14831,-3.22618,-9.76403,-1.7672,4.02369]
optimal[22] = [-8.1911,-9.3191,-9.09697,-11.66293,-5.30617,-6.932,1.4129]
optimal[26] = [-4.80245,-6.28632,-9.45279,-4.16555,-7.18907,-1.80434,7.90979]
optimal[30] = [-2.34705,-7.62794,-8.33515,-6.45821,-5.68233,-5.58406,2.95102]
optimal[42] = [-1,-2.50142,-9.79564,-10.4091,-5.07189,-4.17098,7.18936]
optimal[46] = [-1.4297,-1.02,-7.20777,-5.81342,-5.19093,-3.65364,3.33724]
optimal[50] = [-1,-7.14787,-10.51474,-13.77256,-6.31103,-6.39996,2.45626]
optimal[71] = [-4.12121,-6.86167,-3.7588,-6.87258,-3.07297,-8.9567,4.87043]
optimal[72] = [-4.39593,-6.17996,-9.39633,-3.13429,-2.8007,-4.02407,3.75567]

# 100 iterations, 50 particles, 50000 messages
optimal[10] = [-2.86633, -1.00027, -8.67432, -3.59712, -5.61223, -4.86929,  6.70386]
optimal[22] = [-2.32628,  -1.7308,  -10.80852, -13.38486,  -5.24274,  -5.04454,   2.51131]
optimal[26] = [-3.97679,  -5.30299,  -4.21261, -11.76419,  -8.61554,  -1.0,        7.99185]
optimal[30] = [-2.34718, -7.89272, -9.91605, -6.45294, -5.67398, -5.53205,  2.9375 ]
optimal[42] = [ -1.0   ,    -2.38373, -11.0,      -11.57977,  -5.07323,  -4.1789,    7.18237]
optimal[46] = [-9.59715,  -7.62929,  -7.17666, -11.39635,  -5.81599,  -4.22682,   3.33722]
optimal[50] = [-1.0    ,   -7.08436, -11.0,      -13.80001,  -6.29855,  -6.3265,    2.44553]
optimal[71] = [-6.48202,  -5.11268,  -3.74055, -11.513,    -3.72873,  -4.14102,   4.9738 ]
optimal[72] = [-8.85985, -5.54803, -5.9536,  -6.24368, -3.43682, -8.98123,  3.25082]

optimal[10] = [-8.17488, -1.0,      -3.15327, -4.59592, -2.79217, -7.52512,  4.93032]
optimal[22] = [-4.49302,  -5.21155, -11.0,      -10.31543,  -3.81758,  -3.02599,   2.51746]
optimal[26] = [-9.58377, -7.44585, -8.09891, -4.62427, -9.00994, -2.13645,  8.15214]
optimal[30] = [-1.6291,  -7.78775, -9.20491, -5.35964, -5.43332, -9.64206,  3.50074]
optimal[42] = [-1.0,       -2.31452,  -9.54804, -14.16304,  -9.9707,   -1.0,        5.86915]
optimal[46] = [-8.1844,  -1.00198, -7.15537, -4.05591, -6.83262, -1.26905,  2.6491 ]
optimal[50] = [-1.0,       -7.03234,  -9.5072,  -13.57704,  -3.64051,  -2.23505,   2.7053 ]
optimal[71] = [-6.27799,  -8.00608,  -5.14811, -12.83466,  -4.3171,   -3.96607,   4.80787]
optimal[72] = [-6.91627, -5.24413, -6.18029, -5.87905, -3.35767, -4.09938,  3.06021]

for o in optimal:
    if type(o) is list:
        for j in range(len(o)):
            if j in range(0, 6):
                o[j] = 10**o[j]
