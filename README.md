# ERC Rover Steering Controller

This project provides Pi Pico micro-ROS firmware for controlling **four steering motors** using dual-PWM H-bridge drivers and **AS5048A encoders in PWM mode**. PID control is used to track angle targets per motor, and the controller is fully configurable over ROS 2.

---

## ROS 2 Topics

### Subscribed Topics

#### `/target_angles` (`std_msgs/Float32MultiArray`)

Set target angle in degrees for each motor:
```bash
ros2 topic pub /target_angle std_msgs/msg/Float32MultiArray "data: [10.0, 20.0, 30.0, 40.0]"
```

#### `/pid_params` (`std_msgs/Float32MultiArray`)

Set PID constants. These are automatically saved to EEPROM.
```bash
ros2 topic pub /pid_params std_msgs/msg/Float32MultiArray "{data: [1.2, 0.1, 0.01]}"
```

#### `/zero_offsets` (`std_msgs/Float32MultiArray`) 

Set zero-offsets per motor in degrees. Offsets are also saved to EEPROM.
```bash
ros2 topic pub /zero_offsets std_msgs/msg/Float32MultiArray "{data: [10.0, -3.5, 0.0, 1.2]}"
```
#### `/estop` (`std_msgs/bool`) 

Enable or disable estop (sets target position to current position).
```bash
ros2 topic pub /estop std_msgs/msg/Bool "{data: true}"
```
---

### Published Topics
#### `/current_angles` (`std_msgs/Float32MultiArray`)  
Reads the current angles of the motors as a simple array
```bash
ros2 topic echo /current_angles
```

#### `/joint_states` (`sensor_msgs/JointState`)
Reads the joint states in an RViz compatible output (in radians).

## micro-ROS for PlatformIO

This library is used to communicate with the rest of the system:
[https://github.com/micro-ROS/micro\_ros\_platformio](https://github.com/micro-ROS/micro_ros_platformio)

---

## Setup

### Dependencies

Run the following to ensure you have the correct packages:

```bash
sudo apt update
sudo apt install python3-venv cmake gcc g++
```

Install the PlatformIO extension for VSCode.
**Note:** micro-ROS PlatformIO currently only builds reliably on Ubuntu / Debian / Linux Mint. Windows builds are not supported.

---
