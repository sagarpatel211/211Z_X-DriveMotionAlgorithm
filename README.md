# 211Z X-Drive Motion Algorithm
This is our motion algorithm for an x-drive using odometry and 3 PID feedback loops! It allows us to create precise autonomous routines with simultaneous x, y, and angle motions.


## Table of Contents
* [Dependencies](#dependencies)
* [Installation](#installation)
* [Features](#features)
* [How To Use](#how-to-use)
* [Contributors](#contributors)
* [Contact](#contact)


## Dependencies
[VexCode Pro V5 Text 2.0.0 or later](https://www.vexrobotics.com/vexcode-download)


## Installation
* Make sure all the dependencies are installed
* Download the files
  * Option 1: 🍴 Fork this repository!
  * Option 2: 🧪 Clone the repository to your local machine using https://github.com/sagarpatel211/211Z_X-DriveMotionAlgorithm.git!
* Open *211XDriveMotionAlgo.v5code* in VexCode to open the program
* Download the program to the brain by connecting the V5 Brain or controller to the device via micro-USB and select *download*. In both options, the V5 Brain must be on!
* Run the program by selecting it from the V5 Brain or pressing the *play* button in VexCode **if** the V5 Brain or controller is attached to the device via micro-USB.


## Features
* Contains odometry for robot position tracking
* X, Y, and angle PID function for autonomous (includes **two** options: with or without timescale factored into derivative)
* MoveToPos function that combines all three functions above for easy programming
* Motor value normalization for better robot movements
* Variable reset function for all PIDs
* Included autonomous and joystick/driver control


## How To Use
We have a single function that allows the robot to change in x, y, and angle simultaneously. This function has an argument for each one so you can decide how you want the robot to move.
```
MoveToPos(x-value, y-value, angle-value, kP for x, kI for x, kD for x, kP for y, kI for y, kD for y, kP for angle, kI for angle, kD for angle); 
```
**Example:**
```
MoveToPos(10, 10, 90, 0, 0, 0, 0, 0, 0, 0, 0, 0); //Example: The robot will move 10 inches in the x and y direction (cartesian plane) and turn to 90 degrees (not accounting for PID values)
```


## Contributors
| <a href="https://github.com/sagarpatel211" target="_blank">**Sagar Patel**</a> | <a href="http://github.com/saurinpatel20" target="_blank">**Saurin Patel**</a> |
| :---: |:---:|
| [![Sagar Patel](https://avatars1.githubusercontent.com/u/34544263?s=200)](https://github.com/sagarpatel211)    | [![Saurin Patel](https://avatars3.githubusercontent.com/u/62221622?s=200)](http://github.com/saurinpatel20) |
| <a href="https://github.com/sagarpatel211" target="_blank">`github.com/sagarpatel211`</a> | <a href="http://github.com/saurinpatel20" target="_blank">`github.com/saurinpatel20`</a> |


## Contact
[Email](mailto:patelsag@students.dsbn.org) | [Website](https://sagarpatel211.github.io/)
