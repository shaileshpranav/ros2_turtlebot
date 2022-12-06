# ros2_turtlebot

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

## Overview
Turtlebot3 Simulation using Gazebo in ROS2 for course ENPM 808x 

## Dependencies
- Ubuntu 20.0 or above
- ROS2 Humble/Foxy

## Build
- Create a workspace
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
- Clone the repository
```
git clone https://github.com/shaileshpranav/ros2_turtlebot.git
```
- Build the workspace
```
cd ~/ros2_ws/src
colcon build --packages-select ros2_turtlebot
cd .. && . install/setup.bash
```

- Set turtlebot3 variable for model

```
echo  "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
```
### Run Instructions

- Without ROS Bag record

  ```
  . install/setup.bash
  ros2 launch ros2_turtlebot gazebo.py
  ```

- Record with ROS Bag

  ```
  . install/setup.bash
  ros2 launch ros2_turtlebot gazebo.py record:=True
  ```

- To view ros_bag info

  ```
  ros2 bag info ros2_turtlebot_bag
  ```

### To play from ROS Bag

- On a new terminal

  ```
  cd ~/ros2_ws
  . install/setup.bash
  ros2 run ros2_turtlebot gazebo.py
  ```

- Open another new terminal

  ```
  cd ~/ros2_ws
  . install/setup.bash
  ros2 bag play ros2_turtlebot_bag
  ```

### Output

![output](results/Output.gif)