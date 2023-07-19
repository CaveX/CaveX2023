This folder contains all of the packages to run OpenSHC on the CaveX robot.

# Folder Description
**Dynamixel_interface**

**Dynamixel_joint_state_publisher**

**frankenX_syropod**

The frankenX_syropod package is useful for simulating robots running OpenSHC in RViz.
It is conventionally used by the CSIRO for simulation of the FrankenX syropod, however, the *hexapod21.yaml* `(location: frankenX_syropod > config > hexapod21.yaml)` configuration file can be used to simulate the caveX robot in RViz.
This package does not yet contain a gazebo launch file.

To run the Rviz simulation with the caveX robot run the following:

`roslaunch frankenX_syropod frankenX_highlevel.launch rviz:=true config:=hexapod22`

To move the robot, a control input is required. If you are running the above command on a linux OS, such as the Jetson, a USB connected playstation controller works well for robot control. However if you are running this command on a Windows machine with an embedded linux OS via Windows Subsystem for Linux (WSL) then using a usb control is likely not to work. WSL 2 no longer supports USB inputs, an alternative is to use the in-built rqt control window. This can interface with openSHC via the *syropod_rqt_reconfigure_control* package with is included in this workspace. Hence to launch the RViz simulation of the CaveX robot on WSL the following command  is recommended:

`roslaunch frankenX_syropod frankenX_highlevel.launch rviz:=true config:=hexapod22 control:=rqt`

Note that WSL also does not support linux Graphical User Interfaces (GUIs) including those integrated with ROS, you should look at X11 forwarding for ROS applications if this command does not produce an RViz window.

**hexapod22**

**syropod_highlevel_controller**

**syropod_remote**
