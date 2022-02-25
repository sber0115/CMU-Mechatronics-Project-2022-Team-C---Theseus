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

### Cyberphysical Architecture
#### Hardware Architecture
Once we reach our minimum viable product, we will look into implementing a power conditioning circuit that can appropriately power the 3.3V, 5V, and 12V devices in our ShipBot. We will start off using power through a tether that is similarly conditioned. We will be using two MCUs, and the Jetson Nano will act as the Master in the UART communications that need to take place in order to facilitate the transmission of speed commands to the wheel motors, actuation commands to the linear actuators, and other commands to the end-effector motors and gripper motor.

The BLE Quad boards have built-in motor drivers that use PWM signals from the built-in STM32 microcontroller. To ensure that we can maneuver the robot smoothly throughout the testbed, the wheel motors are coupled with encoders to make sure we can implement closed-loop control.

Figure 4: Hardware Architecture
(Note: that the batteries are powered through the MIPI interface connection)
### Software Architecture

![Software Architecture](/iamges/SoftwareArchitecture.png "Software Architecture Flowchart")

Figure 6: Fiduciary Marker

The block diagram in Figure 6 describes the approximate architecture of the autonomous software we plan on implementing for the ShipBot. The autonomous action begins by processing the RGB camera data and using this data to scan the testbed from our initial position, in search of a fiduciary of our choice on the testbed. Currently the plan is to use the bracket in Figure 6 as the fiduciary, as it is the most distinct and discernible part of the testbed that does not change position with different testbed configurations. If the fiduciary is not in frame, the robot will turn in search of the fiduciary, and will continue to turn until it is detected in frame. The detection and localization of this fiduciary will be handled either by a CNN or more conventional image processing methods to extract the bracket from the whole image.

Once the fiduciary has been located, the robot will use its relative position to estimate its state and plan a path toward the fiduciary, so that it can orient itself against the guide rail and begin its command sequence. Using its estimated state once it reaches the guide rail, the robot will plan a path to the first location in the command file and begin localization of the device. This will involve first classifying the object using a CNN in order to determine whether or not the robot is in the correct location, as well as the configuration of the device (valve open or closed, switch on or off, etc.). The Perspective-n-Point algorithm will then be used to determine the position of the object relative to the camera, and in turn relative to the wheel base. A path will be determined given this relative position, and inverse kinematics equations will be used to determine the configuration necessary to achieve this path. The desired end location will be dependent on the type and configuration of the device (e.g. 10cm above the horizontally oriented lever valve). Motor commands will be sent to the STM32, which will handle low level motor control. The state estimation and planned path will be updated periodically to ensure rocking from the ship has not altered the position of the robot. 

Once the end effector reaches the desired position, a hard-coded control sequence will be executed in order to manipulate the device. We intend on using a control sequence for each device that functions either generically for any configuration of the object or determines from the original CNN classification a modified control sequence.

Once executed, the robot returns to its default position and runs the classification CNN again on the object in order to verify it is in the correct configuration (if possible). It then proceeds to the location specified in the next command and repeats this process until all commands have been executed. 

