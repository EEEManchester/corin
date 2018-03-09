[ control info ]
control_cycle = 10   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE | DEFAULT JOINT
/dev/ttyUSB0 | 1000000  | lf_q1
/dev/ttyUSB0 | 1000000  | lf_q2
#/dev/ttyUSB0 | 1000000  | lf_q3

#/dev/ttyUSB0 | 1000000  | lm_q1

/dev/ttyUSB0 | 1000000  | lr_q1
/dev/ttyUSB0 | 1000000  | lr_q2
/dev/ttyUSB0 | 1000000  | lr_q3

/dev/ttyUSB0 | 1000000  | rf_q1
/dev/ttyUSB0 | 1000000  | rf_q2
/dev/ttyUSB0 | 1000000  | rf_q3

/dev/ttyUSB0 | 1000000  | rm_q1
/dev/ttyUSB0 | 1000000  | rm_q2
/dev/ttyUSB0 | 1000000  | rm_q3

/dev/ttyUSB0 | 1000000  | rr_q1
/dev/ttyUSB0 | 1000000  | rr_q2
/dev/ttyUSB0 | 1000000  | rr_q3

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL    | PROTOCOL | DEV NAME   | BULK READ ITEMS
dynamixel | /dev/ttyUSB0 | 1   | MX-64p2  | 2.0      | lf_q1      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 2   | MX-64p2  | 2.0      | lf_q2      | present_position, present_velocity, present_current
#dynamixel | /dev/ttyUSB0 | 3   | MX-64p2  | 2.0      | lf_q3      | present_position, present_velocity, present_current

#dynamixel | /dev/ttyUSB0 | 4   | MX-64p2  | 2.0      | lm_q1      | present_position, present_velocity, present_current

dynamixel | /dev/ttyUSB0 | 7   | MX-64p2  | 2.0      | lr_q1      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 8   | MX-64p2  | 2.0      | lr_q2      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 9   | MX-64p2  | 2.0      | lr_q3      | present_position, present_velocity, present_current

dynamixel | /dev/ttyUSB0 | 10  | MX-64p2  | 2.0      | rf_q1      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 11  | MX-64p2  | 2.0      | rf_q2      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 12  | MX-64p2  | 2.0      | rf_q3      | present_position, present_velocity, present_current

dynamixel | /dev/ttyUSB0 | 13  | MX-64p2  | 2.0      | rm_q1      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 14  | MX-64p2  | 2.0      | rm_q2      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 15  | MX-64p2  | 2.0      | rm_q3      | present_position, present_velocity, present_current

dynamixel | /dev/ttyUSB0 | 16  | MX-64p2  | 2.0      | rr_q1      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 17  | MX-64p2  | 2.0      | rr_q2      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 18  | MX-64p2  | 2.0      | rr_q3      | present_position, present_velocity, present_current
