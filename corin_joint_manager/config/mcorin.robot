[ control info ]
control_cycle = 10   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE | DEFAULT JOINT
/dev/ttyUSB0 | 1000000  | lf_q1_joint
/dev/ttyUSB0 | 1000000  | lf_q2_joint
/dev/ttyUSB0 | 1000000  | lf_q3_joint

/dev/ttyUSB0 | 1000000  | lm_q1_joint
/dev/ttyUSB0 | 1000000  | lm_q2_joint
/dev/ttyUSB0 | 1000000  | lm_q3_joint

/dev/ttyUSB0 | 1000000  | lr_q1_joint
/dev/ttyUSB0 | 1000000  | lr_q2_joint
/dev/ttyUSB0 | 1000000  | lr_q3_joint

/dev/ttyUSB1 | 1000000  | rf_q1_joint
/dev/ttyUSB1 | 1000000  | rf_q2_joint
/dev/ttyUSB1 | 1000000  | rf_q3_joint

/dev/ttyUSB1 | 1000000  | rm_q1_joint
/dev/ttyUSB1 | 1000000  | rm_q2_joint
/dev/ttyUSB1 | 1000000  | rm_q3_joint

/dev/ttyUSB1 | 1000000  | rr_q1_joint
/dev/ttyUSB1 | 1000000  | rr_q2_joint
/dev/ttyUSB1 | 1000000  | rr_q3_joint

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL    | PROTOCOL | DEV NAME         | BULK READ ITEMS
dynamixel | /dev/ttyUSB0 | 1   | MX-64p2  | 2.0      | lf_q1_joint      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 2   | MX-64p2  | 2.0      | lf_q2_joint      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 3   | MX-64p2  | 2.0      | lf_q3_joint      | present_position, present_velocity, present_current

dynamixel | /dev/ttyUSB0 | 4   | MX-64p2  | 2.0      | lm_q1_joint      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 5   | MX-64p2  | 2.0      | lm_q2_joint      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 6   | MX-64p2  | 2.0      | lm_q3_joint      | present_position, present_velocity, present_current

dynamixel | /dev/ttyUSB0 | 7   | MX-64p2  | 2.0      | lr_q1_joint      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 8   | MX-64p2  | 2.0      | lr_q2_joint      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB0 | 9   | MX-64p2  | 2.0      | lr_q3_joint      | present_position, present_velocity, present_current

dynamixel | /dev/ttyUSB1 | 10  | MX-64p2  | 2.0      | rf_q1_joint      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB1 | 11  | MX-64p2  | 2.0      | rf_q2_joint      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB1 | 12  | MX-64p2  | 2.0      | rf_q3_joint      | present_position, present_velocity, present_current

dynamixel | /dev/ttyUSB1 | 13  | MX-64p2  | 2.0      | rm_q1_joint      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB1 | 14  | MX-64p2  | 2.0      | rm_q2_joint      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB1 | 15  | MX-64p2  | 2.0      | rm_q3_joint      | present_position, present_velocity, present_current

dynamixel | /dev/ttyUSB1 | 16  | MX-64p2  | 2.0      | rr_q1_joint      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB1 | 17  | MX-64p2  | 2.0      | rr_q2_joint      | present_position, present_velocity, present_current
dynamixel | /dev/ttyUSB1 | 18  | MX-64p2  | 2.0      | rr_q3_joint      | present_position, present_velocity, present_current
