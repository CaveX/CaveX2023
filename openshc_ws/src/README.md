This folder contains all of the packages to run OpenSHC on the CaveX robot.

# Folder Description
**Dynamixel_interface**

**Dynamixel_joint_state_publisher**

**frankenX_syropod**

The frankenX_syropod package is useful for simulating robots running OpenSHC in RViz.
It is conventionally used by the CSIRO for simulation of the FrankenX syropod, however, the *hexapod22.yaml* `(location: frankenX_syropod > config > hexapod22.yaml)` configuration file can be used to simulate the caveX robot in RViz.
This package does not yet contain a gazebo launch file.

To run the Rviz simulation with the caveX robot run the following:

`roslaunch frankenX_syropod frankenX_highlevel.launch rviz:=true config:=hexapod22`

**hexapod22**

**syropod_highlevel_controller**

**syropod_remote**
