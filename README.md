# Repository for ENPM 700 ROS2 Gazebo Assignments

### Dependencies/ Assumptions/ Tools used in project

- OS: Ubuntu:22.04
- Developed and tested on x86-64 architecture
- Turtlebot3
- ros2-pkg-create (https://github.com/ika-rwth-aachen/ros2-pkg-create)
- ROS2 - humble (https://docs.ros.org/en/humble/Installation.html)
- python3-colcon-clean (sudo apt-get install python3-colcon-clean -y)
- terminator
- Editor used - VSCODE (https://code.visualstudio.com/)

Simple walker algorithm similar to a Roomba robot vacuum cleaner. Moves forward until encounters an obstacle and then
rotate to again continue.

### Repository directory structure:

```bash

.
├── LICENSE
├── results
│   ├── rosbag2_2024_11_23-22_38_19
│   │   ├── metadata.yaml
│   │   └── rosbag2_2024_11_23-22_38_19_0.db3
│   └── state_diagram.svg
├── this_is_inside_workspace_src
└── walker
    ├── CMakeLists.txt
    ├── config
    │   └── params.yml
    ├── include
    │   └── walker
    │       └── walker_node.hpp
    ├── launch
    │   ├── turtlebot3_maze.launch.py
    │   └── walker_demo.launch.py
    ├── models
    ├── package.xml
    ├── README.md
    ├── src
    │   └── walker_node.cpp
    └── worlds
        └── maze.world

```

## State Diagram

![image](https://github.com/Arthav24/my_gazebo_tutorials/blob/working_with_gazebo/results/state_diagram.svg)

Context - WalkerNode<br>
StateInterface Class - State<br>
Concrete Classes - IDLE, MoveForward, Rotate

TurtleBot3 scan for obstacle in ranges [10deg,0deg] and [360deg,350deg] this having 20deg obstacle sensing zone in front
of the bot.
<br>
TurtleBot3 changes direction(CW, CCW) in Rotate State alternatively. This functionality has been clubbed in to Rotate
state with a variable pointing next direction of rotation.

Start State of Walker is IDLE.

## Directory setup

``` bash
mkdir -p ~/gazebo_ws/src
cd ~/gazebo_ws/src
git clone https://github.com/Arthav24/my_gazebo_tutorials.git
```

Install required dependencies

``` bash 
rosdep update
rosdep install --from-path src --ignore-src -y # assuming ROS_DISTRO is set to humble
```

## How to Compile:

```bash
cd ~/colcon_ws/
colcon clean workspace
source /opt/ros/humble/setup.bash  # if needed
colcon build
```

## How to Run:

```bash
source install/setup.bash

export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=${PWD}/install/walker/share/walker/models/

# on one terminal launch turtlebot3 in maze
source install/setup.bash
ros2 launch walker turtlebot3_maze.launch.py

# on second terminal launch walker 
source install/setup.bash
ros2 launch walker walker_demo.launch.py record_bag:=false # To record ros2 bag switch to true

# On third terminal call /start service
source install/setup.bash
ros2 service call /start std_srvs/srv/Trigger {}
#This will change state to MoveForward

# To stop the robot 
ros2 service call /stop std_srvs/srv/Trigger {}
```

## Bag info

```bash
Opened database 'src/my_gazebo_tutorials/rosbag2_2024_11_23-22_38_19/rosbag2_2024_11_23-22_38_19_0.db3' for READ_ONLY. ---------- (open() at ./src/rosbag2_storage_default_plugins/sqlite/sqlite_storage.cpp:228)

Files:             src/my_gazebo_tutorials/rosbag2_2024_11_23-22_38_19/rosbag2_2024_11_23-22_38_19_0.db3
Bag size:          5.1 MiB
Storage id:        sqlite3
Duration:          51.406884780s
Start:             Nov 23 2024 22:38:19.751440989 (1732419499.751440989)
End:               Nov 23 2024 22:39:11.158325769 (1732419551.158325769)
Messages:          12412
Topic information: Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 257 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 1 | Serialization Format: cdr
                   Topic: /robot_description | Type: std_msgs/msg/String | Count: 1 | Serialization Format: cdr
                   Topic: /parameter_events | Type: rcl_interfaces/msg/ParameterEvent | Count: 1 | Serialization Format: cdr
                   Topic: /performance_metrics | Type: gazebo_msgs/msg/PerformanceMetrics | Count: 254 | Serialization Format: cdr
                   Topic: /joint_states | Type: sensor_msgs/msg/JointState | Count: 1370 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 22 | Serialization Format: cdr
                   Topic: /odom | Type: nav_msgs/msg/Odometry | Count: 1370 | Serialization Format: cdr
                   Topic: /imu | Type: sensor_msgs/msg/Imu | Count: 5883 | Serialization Format: cdr
                   Topic: /events/write_split | Type: rosbag2_interfaces/msg/WriteSplitEvent | Count: 0 | Serialization Format: cdr
                   Topic: /cmd_vel | Type: geometry_msgs/msg/Twist | Count: 514 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 2236 | Serialization Format: cdr
                   Topic: /clock | Type: rosgraph_msgs/msg/Clock | Count: 503 | Serialization Format: cdr
```

## Node Details

## `walker_node`

### Launch Files

| File                      | Description                          |
|---------------------------|--------------------------------------|
| turtlebot3_maze.launch.py | Launch file for turtlebot3 with maze |
| walker_demo.launch.py     | Launch file for walker node          |

### Subscribed Topics

| Topic             | Type                              | Description |
|-------------------|-----------------------------------|-------------|
| /parameter_events | rcl_interfaces/msg/ParameterEvent |             |
| /scan             | sensor_msgs/msg/LaserScan         |             |

### Published Topics

| Topic             | Type                              | Description                          |
|-------------------|-----------------------------------|--------------------------------------|
| /cmd_vel          | geometry_msgs/msg/Twist           | Command velocity topic of turtlebot3 |
| /parameter_events | rcl_interfaces/msg/ParameterEvent | Param event handler                  |

### Services

| Service | Type                 | Description                                                       |
|---------|----------------------|-------------------------------------------------------------------|
| /start  | std_srvs/srv/Trigger | Service to Start the robot or transition from IDLE to MoveForward |
| /end    | std_srvs/srv/Trigger | Service to Stop the robot or transition to IDLE                   |

### Parameters

| Parameter         | Type                              | Description         |
|-------------------|-----------------------------------|---------------------|
| /parameter_events | rcl_interfaces/msg/ParameterEvent | Param event handler |


