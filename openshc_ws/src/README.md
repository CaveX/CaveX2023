This folder contains all of the packages to run OpenSHC on the CaveX robot.

# Folder Description
**Dynamixel_interface**

This ROS package is used to interface and control the Dynamixel servo motors on the caveX robot, refer to this [link](https://github.com/csiro-robotics/dynamixel_interface).

**Dynamixel_joint_state_publisher**

The Dynamixel joint state publisher is a ROS node written in python that is subscribed to the joint state of the Dynamixel motors in each joint of the robot's legs and publishes to a state topic which is subscribed to by the syropod highlevel controller (OpenSHC), see this [link](https://github.com/csiro-robotics/dynamixel_joint_state_publisher).

**frankenX_syropod**

The frankenX_syropod package is useful for simulating robots running OpenSHC in RViz, see this [link](https://github.com/csiro-robotics/frankenX_syropod/tree/master).
It is conventionally used by the CSIRO for simulation of the FrankenX syropod, however, the *hexapod21.yaml* `(location: frankenX_syropod > config > hexapod21.yaml)` configuration file can be used to simulate the caveX robot in RViz.
This package does not yet contain a gazebo launch file.

To run the Rviz simulation with the caveX robot run the following:

`roslaunch frankenX_syropod frankenX_highlevel.launch rviz:=true config:=hexapod22`

To move the robot, a control input is required. If you are running the above command on a linux OS, such as the Jetson, a USB connected playstation controller works well for robot control. However if you are running this command on a Windows machine with an embedded linux OS via Windows Subsystem for Linux (WSL) then using a usb control is likely not to work. WSL 2 no longer supports USB inputs, an alternative is to use the in-built rqt control window. This can interface with openSHC via the *syropod_rqt_reconfigure_control* package with is included in this workspace. Hence to launch the RViz simulation of the CaveX robot on WSL the following command  is recommended:

`roslaunch frankenX_syropod frankenX_highlevel.launch rviz:=true config:=hexapod22 control:=rqt`

Note that WSL also does not support linux Graphical User Interfaces (GUIs) including those integrated with ROS, you should look at X11 forwarding for ROS applications if this command does not produce an RViz window.

**hexapod22**

This repository contains code for simulating the CaveX robot in RViz and Gazebo however does not work correctly, see this [link](https://github.com/CaveX/hexapod22/blob/main/README.md).

**syropod_highlevel_controller**

This code repository was created by the CSIRO and is responsible for generating gaits and poses for multilegged robots, see this [link](https://github.com/csiro-robotics/syropod_highlevel_controller).

**syropod_remote**

This repository acts as the interface between user input devices and the *syropod_highlevel_controller*, see this [link](https://github.com/csiro-robotics/syropod_remote).

**syropod_rqt_reconfigure_control**

As stated in the description of the *frankenX_syropod*, the *syropod_rqt_reconfigure_control* repository contains code with supplies an rqt GUI which can interface with OpenSHC for control, syropod_remote can therefore not be running whilst this is in used. See this [link](https://github.com/csiro-robotics/syropod_rqt_reconfigure_control) for more information.

The rqt control method is generally set in a *roslaunch* command by the following syntax, note that the default value for control is joy (joystick):

`control:=rqt`
