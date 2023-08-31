# IMU Ros Node

The selected IMU is the BNO-055. The IMU is powered using the 3.3V pins and communicates on I2C bus 1 on the carrier board. The Jetson Orin itself interprets this communication channel as bus number 7. To verify that the IMU chip is detected in Ubuntu run the command:

```sudo i2cdetect -y -r 7```

This commands probes the address for I2C bus 7 and should print out a chip address at 0x28. Note that libi2c-dev ubuntu package is required for the above command.

## Viewing IMU Data in RVIZ

#### IMU Initalisation & RVIZ Setup
The IMU data was viewed using the IMU rviz plug-in available from ROS. To install the plug-in for Ubuntu:

```$ sudo apt-get install ros-noetic-rviz-imu-plugin```

To be able to view the IMU data the imu node must be launched. For the 2023 CaveX system a startup service and bash script was written to launch this node when the system boots up. The command to launch the node is:

```$ roslaunch imu_bno055 imu.launch```

To verify the data is being published run the command:

```$ rostopic echo /imu/data```

#### RVIZ Data Visualisation
1. Launch rviz

```$ rosrun rviz rviz```

2. Changed the fixed frame selection in the panel on the left to imu (You have to select this text field and manually type 'imu')

3. Click Add button in bottom left of rviz

4. Click imu under rviz_imu_plugin in the first options tab (Not the topics tab) and click OK

5. Change the topic of the added imu to /imu/data

6. Move IMU around to see the axes orientation and associated data updating in real time
