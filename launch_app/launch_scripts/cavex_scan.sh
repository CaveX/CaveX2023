#!/bin/bash
# Need to update this file with new paths
source /opt/ros/melodic/setup.bash

# rosrun velodyne_driver velodyne_node _model:=32E
# sleep 60 # doesn't work. I think it's the permissions
rosrun velodyne_driver vdump pcap-
echo "sl@m!12"
