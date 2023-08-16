#!/bin/bash
# Need to update this file with new paths
source /opt/ros/noetic/setup.bash
#source /home/cavex/interbotix_ws/devel/setup.bash
source /cavex_workspace/dev/CaveX2023/openshc_ws/devel/setup.bash

# start the SHC and dynamixel_interface respectively (hexapod22?)
roslaunch hexapod22 dynamixel_interface_controller.launch &
roslaunch frankenX_syropod frankenX_highlevel.launch config:=hexapod21
