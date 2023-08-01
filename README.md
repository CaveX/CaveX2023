# CaveX2023
All the code for the 2023 iteration of the CaveX project. This repository is an amalgamation of the repositories used in each of the project iterations (2021, 2022 & 2023). As such, this repository contains all the code needed to run the CaveX robot!

## Info
**"catkin_ws" folder**: The Catkin Workspace which contains all the ROS packages

**"launch_scripts" folder**: All the service files which are placed in the /lib/systemd/system directory on the Jetson processor. This contains all the startup services for the Jetson to run the services for ROS and OpenSHC. See the README.md inside the launch scripts folder.

**"openshc_ws" folder**: All the repositories from the CSIRO to help control the robot and visualise it using RViz or Gazebo, see the README.md file in `openshc_ws > src` for a description on the packages utilised by the 2023 team.

TODOs:
- Overview
- Getting started guide
- Repo anatomy documentation
- Useful resources

## Useful Resources
|Description        |Link                          |
|----------------|-------------------------------|
|Using ROS in WSL2/Creating ROS packages and nodes in C++|[Emil Vidmark (9 videos)](https://www.youtube.com/watch?v=C6BlNbeU3fQ)|
