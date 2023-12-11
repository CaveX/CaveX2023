# CaveX2023
All the code for the 2023 iteration of the CaveX project. This repository is an amalgamation of the repositories used in each of the project iterations (2021, 2022 & 2023). As such, this repository contains all the code needed to operate the Arachnida robot, and simulate it in RViz and Gazebo!

## Software Development Process
Each of the 2023 team members (Luka Moran, Riley Groome, Tyler Groome) utilised windows PCs with Windows Subsystem for Linux (WSL) version 2 for software development. WSL 2 does however have considerable drawbacks. These include blocking device inputs such as playstation controllers and running linux GUIs (such as RViz and Gazebo) extremely slowly. To overcome these issues the CaveX designated computer, which runs Ubuntu 20.04 was utilised for Gazebo and RViz simulations. Once code was tested in simulation it was brought across to the Jetson processor for physical prototype testing. If an improvement was observed the code was merged into the main branch and permanently built on the Jetson. The following development process is summarised below:

![DT E](https://github.com/CaveX/CaveX2023/assets/110513531/8a3289da-5f09-4349-ab33-c5103d64f9eb)

## First steps
The Jetson runs Ubuntu 20.04, Noetic is the ROS1 distriution which is targeted at this version of Ubuntu. On your personal PC you should have Ubuntu 20.04 installed (If you're on a windows machine you'll need WSL2).
- Follow these [instructions](https://learn.microsoft.com/en-us/windows/wsl/install) for setting up WSL, and upgrading to WSL2.
- Follow the [ROS wiki](http://wiki.ros.org/noetic/Installation/Ubuntu) instructions to install ROS noetic and its dependencies.

The README on the Open Source Syropod Highlevel Controller (OpenSHC) [git page](https://github.com/csiro-robotics/syropod_highlevel_controller) is also worth looking into. This page features a link to OpenSHC tutorials which demonstrate the steps to build the repository and its dependencies for PC's or Raspberry Pi's (or Jetsons!). These steps have already been followed by the 2023 team so the steps to build the code are mentioned in the README in `openshc_ws > src`.

### Dependencies
The following are required dependencies to run the arachnida ROS package, dependencies for OpenSHC and Syropod Remote should be documented on the appropriate CSIRO git page. Note that these dependencies are already installed on the Jeston Orin NX processor but will need to be installed on your personal computer if you wish to run the Arachnida ROS package. 
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page#Download): C++ library which is used for various linear algebra operations, used in SLAM, obstacle detection, and path-planning.
- [Ceres Solver](http://ceres-solver.org/installation.html): Solver which configures transforms between frames for SLAM, also read through the dependencies documented on the attached link.

## Info
Each folder has its own README.md file with explanations on code and directory structure, and how to run the code.
**"arachida_ws" folder**: The Arachnida workspace which contains all the source code for the Arachida ROS package.

**"launch_scripts" folder**: Contains service files and shell scripts which are located on the Jetson. The service files which are placed in the /lib/systemd/system directory on the Jetson processor and are enabled to run on system bootup. This contains all the startup services for the Jetson to run the services for ROS and OpenSHC. The launch shell scripts are alled by these services and run the commands to start the ROS processes.

**"openshc_ws" folder**: All the repositories from the CSIRO to help control the robot, and visualise it using RViz or Gazebo, see the README.md file in `openshc_ws > src` for a description on the packages utilised by the 2023 team. Note that modifications have been made to the CSIRO's syropod_remote repository for integration with dronedeploy and robust dynamic control method switching.

## Useful Resources
|Description        |Link                          |
|----------------|-------------------------------|
|ROS Wiki Documentation|https://wiki.ros.org/Documentation|
|ROS Tutorials|https://wiki.ros.org/ROS/Tutorials|
|Using ROS in WSL2/Creating ROS packages and nodes in C++|[Emil Vidmark (9 videos)](https://www.youtube.com/watch?v=C6BlNbeU3fQ)|
|OpenSHC Git Page|https://github.com/csiro-robotics/syropod_highlevel_controller|
|OpenSHC Paper|https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9223640|
|Syropod Remote Git Page|https://github.com/csiro-robotics/syropod_remote|
|Dynamixel Interface|https://github.com/csiro-robotics/dynamixel_interface|
|DroneDeploy Docs|https://docs-automate.dronedeploy.com/|
