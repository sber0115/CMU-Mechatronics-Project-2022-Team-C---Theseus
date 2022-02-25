---
layout: page
title: "System Description"
permalink: /SystemDescription/
---

# System Design
## Design Requirements
Below is a list of requirements derived from the ShipBot specifications given to us as well as our own requirements based on our personal limitations/goals.
- Construction:
-- Size: 1.5’x1.5’x2.5’
-- Power: Portable onboard power source for mobility (LiPo or Li-Ion)
-- Budget: $1000 for purchases
-- Completely autonomous
-- Tight tolerances
-- Strong construction and portable in case robot must be moved to work on in different locations
-- Low center-of-mass with heavy counterweight to avoid tipping
-- Components easily accessible for easy replacement/repair
- Performance:
-- Testbed:
-- 3’x5’ operating area
-- Eight 1’ stations
-- 12”-24” component height
-- Rotary Valve, Spigot Valve, Shuttlecock Valve, Breaker Panel
-- 1-minute setup period
-- Robust to arbitrary start location
-- Robust to rocking of operating area
-- Time: 1min x Number of Stations + 30 x Number of additional devices at station + 20 x Number of Devices (extra time)
-- Parses mission file for given commands
-- Indicates via alarm/sound when finished
-- Robust to loss of state or faulty state estimation
-- Robust to lighting conditions
