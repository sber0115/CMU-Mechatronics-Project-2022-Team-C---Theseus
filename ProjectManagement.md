---
layout: page
title: "ProjectManagement"
permalink: /ProjectManagement/
markdown: kramdown
---

# Project Management
## Schedule
| Week | Progress |
| --- | --- |
| 1 | Initial concept discussion |
| 2 | Physical design consolidation, mock-up and parts selection |
| 3 | CAD development, mechanical and electrical architecture design |
| 4 | Collect CV images, order parts, control logic research |
| 5 | Test parts, robot base frame test assembly |
| 6 | Design refinement, start CV training and initial control implementation  |
| 7 | CV training and camera calibration |
| 8 | Electronic test integration,  inverse kinematics implementation |
| 9 | Redesign parts fabrication and test fitting, CV testing  |
| 10| System integration, assembly and testing with various scenarios |
| 11| Testing and final assembly |


## Team Member Responsibility
| Team Members | Primary Responsibility | Secondary Responsibility |
| --- | --- | --- |
| Dajun Tao | Control/Fabrication | CAD/CAE/CV |
| Ethan Rich | Hardware | Fabrication/Embedded SW |
| Sam Yin | CAD/CAE | Control/CV/Fabrication |
| Sebastian Bernal | Embedded SW | Hardware/Website |
| Valmiki Kothare | Perception/CV | Control/Website |


## Tentative Budget
| Item | Amount | Total Cost | Retailer |
| --- | --- | --- | --- |
| 4040 Hollow Aluminum Extrusions 6 ft | 2 | $141.72 | McMaster-Carr |
| 4040 Series Corner Bracket with scres and nuts | 10 | $18.99 | Amazon.com |
| 4040 Series 3-Way End Corner Bracket | 8 | $27.98 | Amazon.com |
| L16-P Miniature Linear Actuator with Feedback | 2 | $160 | Actuonix |
| Flanged Bearing for T-slotted framing rails | 2 | $114.78 | McMaster-Carr |
| Threaded Round Standoff Aluminum 1/4" OD, 1-1/2" Long, 4-40 Thread Size | 4 | $6.24 | McMaster-Carr |
| Threaded Round Standoff Aluminum 1/4" OD, 2-1/2" Long, 4-40 Thread Size | 6 | $13.38 | McMaster-Carr |
| 4pcs Mechanum wheel & motor set | 1 | $89 | Amazon.com |
| Romeo BLE Quad | 4 | $160 | DF Robot |
| DC 12 V Motor | 4 | $79.60 | Amazon.com |
| 25mm DC Gear Motors Mounting Bracket | 4 | $13.78 | Amazon.com |
| #4-40 Thread Size Screws, 1/4" Length | 100 | $4.91 | Amazon.com |
| #4 Flat Washer | 100 | $7.92 | Amazon.com |
| L298N Motor Driver 4-pack| 1 | $ 11.39 | Amazon.com |
| M8-1.25 x 20mm Screws and Nuts | 15 | $11.49 | Amazon.com |
| 4.5 mm Steel Rod 3-3/8" Long | 3 | $10.83 | Amazon.com |
| 12 Hole Diameter 45# Steel Module 16 Teeth Bevel Gear | 2 | $13.94 | Amazon.com |
| Sum | | $739.76| |

## Risk Management
Due to the limited budget, and the high price of several components, like linear actuators and mecanum wheels, components should have reasonable specifications, and certain less important components should be chosen with lower specifications to reduce cost. For linear actuators, only the vertical part needs to generate high torque, so the horizon linear actuator could have a lower torque payload. Also, due to the size of the testing environment and the stroke length of the linear actuator should be lower than the height of the targets. For mecanum wheels, different load capacity would influence the cost, so choosing lower load capacity would reduce the overall cost. Moreover, using existing components from previous teams could also help to reduce components costs and long parts wait time.
The length of the semester also constrains the complexity of the project, while the high degree of custom design and prototyping demands a long time of researching, developing and testing. In the case of severe time constraints, some integration and ideas from previous years teams should be considered to reduce the development time. Existing products could also alleviate the time constraint. Weekly reports and meetings would be crucial to gauge the progress of the development and to make necessary changes.
