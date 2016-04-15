#!/bin/bash
rosparam set /tactile_servo/id_finger_param 0
rosrun tactile_servoing_shadow	tactile_servo_node __name:=th_servo

rosparam set /tactile_servo/id_finger_param 1
rosrun tactile_servoing_shadow	tactile_servo_node __name:=ff_servo

rosparam set /tactile_servo/id_finger_param 2
rosrun tactile_servoing_shadow	tactile_servo_node __name:=mf_servo

rosparam set /tactile_servo/id_finger_param 3
rosrun tactile_servoing_shadow	tactile_servo_node __name:=rf_servo

rosparam set /tactile_servo/id_finger_param 4
rosrun tactile_servoing_shadow	tactile_servo_node __name:=lf_servo