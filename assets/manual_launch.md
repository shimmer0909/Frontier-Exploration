## 🚀 Launch the Full System Manually

### 1 Start Simulation (Gazebo + sensors)

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
### 2 Launch robot state publisher (URDF).

```bash
ros2 run robot_state_publisher robot_state_publisher   $(ros2 pkg prefix turtlebot3_description)/share/turtlebot3_description/urdf/turtlebot3_waffle.urdf   --ros-args -p use_sim_time:=true
```

### 3 Launch odometry publisher / fake node.

```bash
ros2 run turtlebot3_fake_node turtlebot3_fake_node --ros-args --params-file ~/waffle_fake_params.yaml
```

### 4 Start SLAM

```bash
ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=true
```

### 5 Start Nav2

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```
OR

```bash
ros2 launch nav2_bringup bringup_launch.py \
  params_file:=/home/second-code/Desktop/Robotics/Frontier-Exploration/ros2_ws/src/my_nav2_config/config/nav2_params.yaml \
  use_sim_time:=True \
  slam:=True \
  map:=/tmp/ignore.yaml
```

### 6 Start RViz

```bash
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### 7 Start Frontier Explorer

```bash
ros2 launch frontier_exploration frontier_explorer.launch.py use_sim_time:=true
```
