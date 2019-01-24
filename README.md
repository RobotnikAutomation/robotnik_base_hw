# robotnik_base_hw

ROS RobotHW component based on ros_control architecture. 

This component is compatible with most of the Robotnik's motor hardware.

## Dependencies

* robotnik_base_hw_lib
  * Branch/version: tag kinetic-0.9.11 (binary in lib folder)

* robotnik_msgs 
  * Repository: https://github.com/RobotnikAutomation/robotnik_msgs
  * Branch/version: tag kinetic-0.2.5

* peak-linux-driver
  * Recomended version: 8.5.1 (https://www.peak-system.com/fileadmin/media/linux/files/peak-linux-driver-8.5.1.tar.gz)

## Installation

* Run "sudo dpkg -i lib/ros-kinetic-robotnik-base-hw-lib_X.Y.Z-0xenial_amd64.deb" 

## Auto Recovery

Robot is able to auto recover it's hardware in case of malfunctioning. This is controlled by two parameters:

  **auto_recovery: true**
    \# if true, tries to recover from fatal errors in hw, mostly after pressing e-stop. must have a recovery_period set

  **recovery_period: 10**
    \# period in sec between recovery attempts. if 0, auto_recovery will not work
