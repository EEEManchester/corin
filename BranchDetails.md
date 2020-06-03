# Brief

Used for all wall walking related stuffs
- RViZ setpoint for wall cornering
- Close-loop controllers for wall walking (straight and corner)

ADDENDUM: Above statements unclear

Pre-requisite

    pip install cdd
    git clone https://weicwc@bitbucket.org/weicwc/rviz_visual.git

To run short simulation on Rviz:

    roslaunch corin_description rviz.launch
    rosrun corin_control main.py wall