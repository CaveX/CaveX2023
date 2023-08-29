#!/bin/bash
source /opt/ros/noetic/setup.bash
# Need to update file paths
source /cavex_workspace/dev/CaveX2023/imu_ws/devel/setup.bash

roslaunch imu_bno055 imu.launch
