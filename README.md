# Field Trial Code for 2023. WUR and OSU Collaboration.

## Requirements

### ROS2, Moveit2 and UR Drivers
All development for this project was done in ROS2 Rolling. Other ROS2 distributions may work, but compatability is not guaranteed. 

It is important that the user has a source-based installation of the UR Drivers. The default servo node configuration file will need to be replaced with the version in this repository.

## Current Issues

The robot is currently exhibiting jerky movement, even when sent simple velocity commands at a high frequency. The behavior only occurs on the real hardware, not in simulation.

The tangent selection process for the heuristic controller currently has a bug where it will select sequential tangents that have opposite directions. I have already written some alternative tangent selection logic, but it has not been tested in simulation yet.

