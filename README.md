# Lab 7: Mobile Robot II

2.12/2.120 Intro to Robotics  
Spring 2024[^1]

<details>
  <summary>Table of Contents</summary>

- [1 Mobile Robot](#1-mobile-robot)
  - [1.1 Understand `robot/`](#11-understand-robot)
  - [1.2 Understand Odometry](#12-understand-odometry)
  - [1.3 U-Turn](#13-u-turn)
  - [1.4 Circle](#14-circle)
- [2 Joystick Control](#2-joystick-control)
  - [2.1 Get Robot MAC Address](#21-get-robot-mac-address)
  - [2.2 Get Controller MAC Address](#22-get-controller-mac-address)
  - [2.3 Validate Controller](#23-validate-controller)
  - [2.4 Run Controller](#24-run-controller)
  - [2.5 Run Joystick Control](#25-run-joystick-control)
- [3 Custom Trajectory](#3-custom-trajectory)
- [4 Feedback Form](#4-feedback-form)
- [5 Optional Extensions](#5-optional-extensions)
  - [5.1 IMU](#51-imu)
  - [5.2 Mecanum Wheels](#52-mecanum-wheels)

</details>

We have already written most of the code for this lab. We hope that you will use the extra free time to fully understand the mobile robot codebase and prepare for the final competition. Have a great spring break!

## 1 Mobile Robot

### 1.1 Understand `robot/`

Take some time to understand `robot_main.cpp`, `robot_drive.cpp`, `robot_motion_control.cpp`, and `robot_wireless.cpp`. At a high level:
- `robot_main.cpp`: Includes the `setup()` and `loop()` functions, telling the microcontroller exactly what to do and when.
- `robot_drive.cpp`: Sets up the motors and implements a PI controller to follow velocity setpoints.
- `robot_motion_control.cpp`: Calculates odometry and setpoints based on either joystick or a given trajectory.
- `robot_wireless.cpp`: Sets up two-way wireless communication with and sends messages to the microcontroller on your controller.

### 1.2 Understand Odometry

Open `robot_motion_control.cpp` and read through `updateOdometry()`. Make sure that you understand exactly how this function calculates odometry data. For reference:

<p align="center">
<img src="./.images/odom.png" alt="drawing" width="1000"/>
</p>

### 1.3 U-Turn

Make sure your PlatformIO environment is set to be `env:robot` and upload the code to the microcontroller on your mobile robot. Once the code has finished uploading, unplug the robot from your computer, set it on the ground, power it on, and press `RST`. Your robot should now autonomously follow a U-Turn!

Your U-Turn will probably not be perfect! While odometry is straightforward to implement, it suffers from problems such as position drift due to wheel slippage. [IMU data](#51-imu) (or some combination of IMU and odometry) will likely be more reliable. 

### 1.4 Circle

In `robot_motion_control.cpp`, comment out `#define UTURN` and uncomment `#define CIRCLE`. This will change the `followTrajectory()` function to follow a circle instead of a U-Turn. Your robot should now autonomously follow a circle!

## 2 Joystick Control

### 2.1 Get Robot MAC Address

In order to establish wireless communication, we first have to make sure that both microcontrollers know each other's MAC addresses.

Run `lib/Wireless/examples/get_mac.cpp` (you will have to temporarily move the existing files inside the `src/robot/` folder and replace them with `get_mac.cpp`). Open `lib/Wireless/wireless.h` and change `robotAddr` to the MAC address being printed to the Serial monitor.

### 2.2 Get Controller MAC Address

Connect to the microcontroller on your controller and change your PlatformIO environment to be `env:controller`.

<details> <summary> <i> Forget how to change environments? </i> </summary>

Please refer to the [instructions from Lab 6](
https://github.com/mit212/lab6_2024?tab=readme-ov-file#1-changing-platformio-environment).

</details>

Run `lib/Wireless/examples/get_mac.cpp` (you will have to temporarily move the existing files inside the `src/controller/` folder and replace them with `get_mac.cpp`). Open `lib/Wireless/wireless.h` and change `controllerAddr` to this MAC address.

### 2.3 Validate Controller

Run `src/test_controller/controller_test.cpp`. You should see joystick readings being printed to your Serial monitor.

### 2.4 Run Controller

Upload `controller_main.cpp` and `controller_wireless.cpp` to the microcontroller on your controller. This will read the joystick and set up two-way wireless communication with the microcontroller on the mobile robot.

### 2.5 Run Joystick Control

In `robot_motion_control.cpp`, comment out `#define CIRCLE` and uncomment `#define JOYSTICK`. This will change the `followTrajectory()` function to follow a joystick instead of a circle. 

Set your PlatformIO environment back to `env:robot`. Upload `robot_main.cpp`, `robot_drive.cpp`, `robot_motion_control.cpp`, and `robot_wireless.cpp` to the microcontroller on your mobile robot. At this point, you should be able to drive your mobile robot around with your joystick!

| :white_check_mark: CHECKOFF 1 :white_check_mark:   |
|:---------------------------------------------------|
| Show your mobile robot in action to a TA or LA. |

## 3 Custom Trajectory

In `robot_motion_control.cpp`, comment out `#define JOYSTICK` and uncomment `#define YOUR_TRAJECTORY`. In the `followTrajectory()` function, make your own path using a state machine, taking `UTURN` as inspiration.

| :white_check_mark: CHECKOFF 2 :white_check_mark:   |
|:---------------------------------------------------|
| Show your mobile robot in action to a TA or LA. |

## 4 Feedback Form

Before you leave, please fill out https://tinyurl.com/212-feedback. 

| :white_check_mark: CHECKOFF 3 :white_check_mark:   |
|:---------------------------------------------------|
| Show the feedback form completion screen to a TA or LA. |

## 5 Optional Extensions

### 5.1 IMU 

Integrate the IMU with your mobile robot! This will probably be very useful for your final project.

### 5.2 Mecanum Wheels

Replace the existing wheels with mecanum wheels! Mecanum wheels allow the robot to move in any direction. However, the odometry and controller will be slightly different.


[^1]: Version 1 - 2016: Peter Yu, Ryan Fish and Kamal Youcef-Toumi  
  Version 2 - 2017: Yingnan Cui and Kamal Youcef-Toumi  
  Version 3 - 2019: Jerry Ng  
  Version 4 - 2023: Joseph Ntaimo, Kentaro Barhydt, Ravi Tejwani, Kamal Youcef-Toumi and Harrison Chin  
  Version 5 - 2024: Jinger Chong, Josh Sohn
