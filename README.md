# ROBOTNIK BASE HW

This package is a ROS RobotHW component based on ros_control architecture, compatible with most of the Robotnik's motor hardware.

---

## Dependencies

- robotnik_base_hw_lib [🔗](https://github.com/RobotnikAutomation/robotnik_base_hw_lib/)

```bash
sudo dpkg -i lib/ros-kinetic-robotnik-base-hw-lib_X.Y.Z-0xenial_amd64.deb
```

- robotnik_msgs [🔗](https://github.com/RobotnikAutomation/robotnik_msgs/)

```bash
git clone https://github.com/RobotnikAutomation/robotnik_msgs/
```

- peak-linux-driver
  * Recomended version: [8.6.0](https://www.peak-system.com/fileadmin/media/linux/files/peak-linux-driver-8.6.0.tar.gz)

---

## ROS

### Params

-  **battery_voltage_offset**: Offset aplied to the voltage getted by MotorDrive to correct the error (default: 0)

-  **base_hw_limits**: Joint limits, like max acceleration or max velocity (default: robotnik_base_hw/config/robotnik_base_hw_limits.yaml)

-  **base_hw_config**: Global configuration about the hardware of the robot (default: find robotnik_base_hw/config/robotnik_base_hw.yaml)

Robot is able to auto recover it's hardware in case of malfunctioning. This is controlled by two parameters:

-  **auto_recovery**: If true, tries to auto recover from fatal errors in hw, mostly after pressing e-stop. Must have a recovery_period set (default: true)

-  **recovery_period**: Period in sec between recovery attempts. If 0, auto_recovery will not work (default: 10)

### Topics

#### Publications

- **state**: Publishes an overview of the state of the node.
  - type: robotnik_msgs/State

- **status**: Publishes an overview of the status of the motors.
  - type: robotnik_msgs/RobotnikMotorsStatus

- **voltage**: Publishes the voltage getted by MotorDrive (with an applied offset).
  - type: std_msgs/Float32

- **emergency_stop**: True when the emergency button has been presed (or an error caused the stop of the robot).
  - type: std_msgs/Bool

- **io**: Publishes a list with the state of all inputs and outputs.
  - type: robotnik_msgs/inputs_outputs

#### Services

- **set_digital_output**: Used to set the state of the digital outputs.
  - type: robotnik_msgs/set_digital_output

- **send_to_home**: Used to set the motor position to 0.
  - type: std_srvs/Trigger
