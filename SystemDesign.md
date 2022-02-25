---
layout: page
title: "System Design"
permalink: /SystemDesign/
---

# System Design
## Design Requirements/Performance Specifications
Below is a list of requirements derived from the ShipBot specifications given to us as well as our own requirements based on our personal limitations/goals.
- Construction:
  - Size: 1.5’x1.5’x2.5’
  - Power: Portable onboard power source for mobility (LiPo or Li-Ion)
  - Budget: $1000 for purchases
  - Completely autonomous
  - Tight tolerances
  - Strong construction and portable in case robot must be moved to work on in different locations
  - Low center-of-mass with heavy counterweight to avoid tipping
  - Components easily accessible for easy replacement/repair
- Performance:
  - Testbed:
  - 3’x5’ operating area
  - Eight 1’ stations
  - 12”-24” component height
  - Rotary Valve, Spigot Valve, Shuttlecock Valve, Breaker Panel
  - 1-minute setup period
  - Robust to arbitrary start location
  - Robust to rocking of operating area
  - Time: 1min x Number of Stations + 30 x Number of additional devices at station + 20 x Number of Devices (extra time)
  - Parses mission file for given commands
  - Indicates via alarm/sound when finished
  - Robust to loss of state or faulty state estimation
  - Robust to lighting conditions

## Functional Architecture

## Cyberphysical Architecture
### Hardware Architecture
Once we reach our minimum viable product, we will look into implementing a power conditioning circuit that can appropriately power the 3.3V, 5V, and 12V devices in our ShipBot. We will start off using power through a tether that is similarly conditioned. We will be using two MCUs, and the Jetson Nano will act as the Master in the UART communications that need to take place in order to facilitate the transmission of speed commands to the wheel motors, actuation commands to the linear actuators, and other commands to the end-effector motors and gripper motor.

The BLE Quad boards have built-in motor drivers that use PWM signals from the built-in STM32 microcontroller. To ensure that we can maneuver the robot smoothly throughout the testbed, the wheel motors are coupled with encoders to make sure we can implement closed-loop control.

Figure 4: Hardware Architecture
(Note: that the batteries are powered through the MIPI interface connection)
### Software Architecture

![Software Architecture](/images/SoftwareArchitecture.png)

Figure 5: Software Architecture Flowchart

![Fiduciary Marker](/images/fiduciary.jpg)

Figure 6: Fiduciary Marker

The block diagram in Figure 6 describes the approximate architecture of the autonomous software we plan on implementing for the ShipBot. The autonomous action begins by processing the RGB camera data and using this data to scan the testbed from our initial position, in search of a fiduciary of our choice on the testbed. Currently the plan is to use the bracket in Figure 6 as the fiduciary, as it is the most distinct and discernible part of the testbed that does not change position with different testbed configurations. If the fiduciary is not in frame, the robot will turn in search of the fiduciary, and will continue to turn until it is detected in frame. The detection and localization of this fiduciary will be handled either by a CNN or more conventional image processing methods to extract the bracket from the whole image.

Once the fiduciary has been located, the robot will use its relative position to estimate its state and plan a path toward the fiduciary, so that it can orient itself against the guide rail and begin its command sequence. Using its estimated state once it reaches the guide rail, the robot will plan a path to the first location in the command file and begin localization of the device. This will involve first classifying the object using a CNN in order to determine whether or not the robot is in the correct location, as well as the configuration of the device (valve open or closed, switch on or off, etc.). The Perspective-n-Point algorithm will then be used to determine the position of the object relative to the camera, and in turn relative to the wheel base. A path will be determined given this relative position, and inverse kinematics equations will be used to determine the configuration necessary to achieve this path. The desired end location will be dependent on the type and configuration of the device (e.g. 10cm above the horizontally oriented lever valve). Motor commands will be sent to the STM32, which will handle low level motor control. The state estimation and planned path will be updated periodically to ensure rocking from the ship has not altered the position of the robot. 

Once the end effector reaches the desired position, a hard-coded control sequence will be executed in order to manipulate the device. We intend on using a control sequence for each device that functions either generically for any configuration of the object or determines from the original CNN classification a modified control sequence.

Once executed, the robot returns to its default position and runs the classification CNN again on the object in order to verify it is in the correct configuration (if possible). It then proceeds to the location specified in the next command and repeats this process until all commands have been executed. 

## Mechanical System
Below is the overall mechanical system, which includes base sub-assembly, arm sub-assembly and gripper sub assembly (Figure 7).

Figure 7: Schematic Diagram of Robot Overall Mechanical System
6.1.1 Robot Base Sub-assembly
The base of the robot consists of four mecanum wheels positioned in the conventional configuration and multiple ultrasonic sensors. The mecanum wheels grant the robot a high level of maneuverability, which is crucial for the operation of the robot. One ultrasonic sensor is placed on each of the sides of the rectangular base. The distance reading from the four sensors provide the information to estimate the initial position in the workspace. The side that faces the target panels is equipped with two ultrasonic sensors placed far apart from each other. The pair is intended to keep the robot parallel to the guide rail during operation. A schematic diagram of the robot base is shown below as Figure 8.

Figure 8: Schematic Diagram of Robot Base Sub-assembly
6.1.2 Robot Arm Sub-assembly
A schematic diagram of the robot arm is shown below. The base arm is equipped with a small-scale linear actuator. The base arm actuator moves the upper base arm, which is rigidly connected to the upper forearm, in the z direction. The linear actuator equipped on the forearm will extend the lower forearm in y (perpendicular to the guide rail) direction. The position in the x direction (parallel to the guide rail) is adjusted by the mecanum wheels on the robot base. The camera is mounted on the lower forearm. During operation, the forearm can position the camera closer to the target panel to gain a better field of view and achieve higher accuracy.

Figure 9: Schematic Diagram of the Robot Arm Sub-assembly
6.1.3 Robot Gripper Sub-assembly
The robot gripper sub-assembly is designed to have three degrees of freedom as Figure 10 shows. The first motor of the gripper would open up and close down to grab targets. After that, the second motor would spin to perform operations on the targets. If the target is in horizontal orientation, the third motor would rotate the whole gripper assembly downward to allow the gripper to grasp it. For the gripper design, the top and bottom lips of the gripper would tightly grip a circular valve (Figure 11). High friction material would also be added to the inside of the lips to increase the friction. The center part of the gripper has a channel that goes through the front face. Along with two lips, this channel would be used to grab the lever valve. The top of the gripper is designed to have an extra length to make contact with the breaker box switch.

![Mark I CAD Models](/images/gripper_isometric_view.png)

Figure 10: Schematic Diagram of Robot Gripper Sub-assembly

![Mark I CAD Models](/images/gripper_part_isometric_view.png)

Figure 11: Schematic Diagram of Robot Gripper 

## Power Subsystem
If we get to implement a battery system, we will most likely use multiple LiPo batteries in parallel to ensure loads are distributed evenly amongst the batteries. This will ensure that we can have ShipBot running as long as possible during any testing or demos. The resulting voltage will need to be buck-converted to a lower voltage to ensure that the BLE Boards, and the Jetson Nano, are appropriately powered at 3.3V, and 5V, respectively. We should be able to easily purchase boards that have any necessary DC/DC converters.


