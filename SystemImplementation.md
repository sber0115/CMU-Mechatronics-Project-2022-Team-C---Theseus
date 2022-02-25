---
layout: page
title: "System Implementation"
permalink: /SystemImplementation/
---

# System Implementation
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

Figure 10: Schematic Diagram of Robot Gripper Sub-assembly

Figure 11: Schematic Diagram of Robot Gripper 
# Software Subsystem
The software subsystem follows the design and architecture specified in the [SystemDesign](SystemDesign.md) page. The interface between hardware and software exists via sensor (RGB camera, proximity sensors, motor encoders) and motor APIs. Low level motor control will be handled on the STM32, while image processing, state estimation, and path planning will be handled on the Jetson Nano.
# Power Subsystem
If we get to implement a battery system, we will most likely use multiple LiPo batteries in parallel to ensure loads are distributed evenly amongst the batteries. This will ensure that we can have ShipBot running as long as possible during any testing or demos. The resulting voltage will need to be buck-converted to a lower voltage to ensure that the BLE Boards, and the Jetson Nano, are appropriately powered at 3.3V, and 5V, respectively. We should be able to easily purchase boards that have any necessary DC/DC converters.
