# robotnik_base_hw

ROS controller component based on ros_control architecture. 

This component is compatible with most of the Robotnik's motor hardware.

## Dependencies

* robotnik_msgs 
  * Repository: https://github.com/RobotnikAutomation/robotnik_msgs
  * Branch/version: kinetic-multi-devel (version 0.2.2) 

* robotnik_base_hw_lib
  * Branch/version: kinetic-multi-devel (version 0.8.0)

* peak-linux-driver
  * Working with version 7.15.2 (http://www.peak-system.com/fileadmin/media/linux/files/peak-linux-driver-7.15.2.tar.gz)
  * Recomended version: 8.5.1 (https://www.peak-system.com/fileadmin/media/linux/files/peak-linux-driver-8.5.1.tar.gz)

## Installation

* Run "dpkg -i robotnik_base_hw_lib.deb". This copy the library librobotnik_base_hw.so to /usr/lib and copy the headers into /usr/include/robotnik_base_hw_lib

Or

* Install ros-kinetic-robotnik-base-hw-lib.deb
