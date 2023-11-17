# CaveX2023
All the code for the 2023 iteration of the CaveX project. This repository is an amalgamation of the repositories used in each of the project iterations (2021, 2022 & 2023). As such, this repository contains all the code needed to operate and simulate the CaveX robot in RViz and Gazebo!

## Software Development Process
Each of the 2023 team members (Luka Moran, Riley Groome, Tyler Groome) utilised windows PCs with Windows Subsystem for Linux (WSL) version 2 for software development. WSL 2 does however have considerable drawbacks. These include blocking device inputs such as playstation controllers and running linux GUIs (such as RViz and Gazebo) extremely slowly. To overcome these issues the CaveX designated computer, which runs Ubuntu 20.04 was utilised for Gazebo and RViz simulations. Once code was tested in simulation it was brought across to the Jetson processor for physical prototype testing. If an improvement was observed the code was merged into the main branch and permanently built on the Jetson. The following development process is summarised below:

![DT E](https://github.com/CaveX/CaveX2023/assets/110513531/8a3289da-5f09-4349-ab33-c5103d64f9eb)

## First steps
The Jetson runs Ubuntu 20.04, Noetic is the ROS distriution which is targeted at this version of Ubuntu. On your personal PC you should have Ubuntu 20.04 installed (If you're on a windows machine you'll need WSL2). Follow the [ROS wiki](http://wiki.ros.org/noetic/Installation/Ubuntu) instructions to install ROS noetic and its dependencies.

The README on the Open Source Syropod Highlevel Controller (OpenSHC) [git page](https://github.com/csiro-robotics/syropod_highlevel_controller) is also worth looking into. This page features a link to OpenSHC tutorials which demonstrate the steps to build the repository and its dependencies for PC's or Raspberry Pi's (or Jetsons!). These steps have already been followed by the 2023 team so the steps to build the code are mentioned in the README in `openshc_ws > src`.

### Dependencies
A required dependency to run the arachnida package which contains LiDAR processing, SLAM, obstacle detection, and gait energetics code is the Eiegn C++ library which is used for various linear algebra operations.

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
