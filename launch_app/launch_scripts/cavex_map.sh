#!/bin/bash
source /opt/ros/noetic/setup.bash
# Need to update file paths
source /cavex_workspace/dev/CaveX2023/arachnida_ws/devel/setup.bash

roslaunch arachnida arachnida_main.launch
