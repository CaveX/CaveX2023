#!/bin/bash
# Need to update this file with new paths
source /opt/ros/noetic/setup.bash
rosrun rosserial_python serial_node.py /dev/ttyUSB1
