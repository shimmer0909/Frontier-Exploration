# Errors I Faced

## Nav2 throws 'Robot is out of bounds of the costmap!'
```bash
[1765479882.488445639] [nav2_costmap_2d]: Robot is out of bounds of the costmap!
```

Solution
1. Check TF tree
```bash
ros2 run tf2_tools view_frames
```
Ensure you have:
1. map -> odom -> base_link
2. odom -> base_footprint -> base_link
3. base_link -> base_scan

if odom → base_footprint → base_link → base_scan
but no odom → base_link

1. create a custom nav2_params.yaml file in src/my_mav2_config/config
2. Integrate this in frontier_explorer.launch.py

## ${namespace} issue
TF frames are being published with a namespace. So tf2_echo base_link base_scan fails if namespace is not included.
Add below after launching Gazebo and before SLAM
1. Publish Waffle URDF + TF
2. Start fake odometry (publish /odom)

## As soon as RViz is started the robot started moving at high frequency

The chain of events :-

1. RViz loads nav2_default_view.rviz
2. That RViz config contains Nav2 panels
3. Those panels activate Nav2 behaviors
4. Nav2 sees that Robot pose is unstable and Map is changing rapidly (SLAM)
5. No valid goal → recovery behaviors kick in
6. Controller starts publishing /cmd_vel at high rate

🧪 How to confirm the real cause
If you see messages without sending any goal → Nav2 recovery is firing.
```bash
ros2 topic echo /cmd_vel
```

Fix :-
1. Stop Nav2 from moving the robot automatically
```bash
# For simulator
pkill -f nav2
# OR
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"
```

2. Disable recovery behaviors temporarily (Prevent Recovery motions)
In nav2_params.yaml:
```yaml
behavior_server:
  ros__parameters:
    enabled: false
```

3. Reduce controller frequency (Prevents Oscillations)
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 5.0   # default is 20.0
```

4. Set correct robot base frame (Prevents Pose drift)
Our TF is map → odom → base_footprint → base_link → base_scan.
So Nav2 must use base_footprint, not base_link:
```yaml
bt_navigator:
  ros__parameters:
    robot_base_frame: base_footprint

controller_server:
  ros__parameters:
    robot_base_frame: base_footprint

local_costmap:
  robot_base_frame: base_footprint

global_costmap:
  robot_base_frame: base_footprint
```

5. Don’t start RViz with Nav2 panels (Prevents Controller activation triggers)

Use rviz2 instead of rviz2 -d nav2_default_view.rviz
Then manually add:
1. Map
2. TF
3. LaserScan

🚫 Do NOT add:
1. Nav2 Goal
2. Nav2 Panel
3. Waypoint Panel (yet)

## After executing Rviz using Rviz2 commad the world starts moving

After previous fix Nav2 stops publishing to /cmd_vel
```bash
# Publisher count should be 0
ros2 topic info /cmd_vel -v
```

🧠 What is actually happening (core concept)
slam_toolbox published from map -> odom that keeps moving the map
```bash
ros2 topic info /tf -v
```
If published count > 0 kill slam_toolbox

```bash
ros2 node list | grep slam
pkill -f slam_toolbox
pkill -f lifecycle_manager_slam
```
The robot should stop moving after this confirming the hypothesis.

Solution - This is NOT a bug. This is correct TF behavior.

🟢 OPTION 1 (RECOMMENDED): Keep SLAM running, but stop Nav2 from driving

This is what you want 99% of the time.

✔ What stays running
1. slam_toolbox ✅
2. diff_drive ✅
3. robot_state_publisher ✅

❌ What must NOT publish velocity
1. controller_server
2. bt_navigator
3. waypoint_follower

Launch Nav2 without activating controllers:
```bash
ros2 launch nav2_bringup bringup_launch.py \
  params_file:=/home/second-code/Desktop/Robotics/Frontier-Exploration/ros2_ws/src/my_nav2_config/config/nav2_params.yaml \
  use_sim_time:=True \
  slam:=False \
  autostart:=False
```
Then activate ONLY what you need:
```bash
ros2 lifecycle set /planner_server configure
ros2 lifecycle set /planner_server activate
```

🟢 OPTION 2: Publish a static map → odom (ONLY for debugging)

⚠️ Not for real SLAM or exploration
```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 map odom
```

1. ✔ Map won’t move
2. ✔ Robot won’t move
3. ❌ SLAM disabled
4. ❌ Map frozen

Use only for visualization checks.

🟢 OPTION 3: Freeze SLAM (advanced, correct way)

If you want:

1. map stays visible

2. no TF jitter

3. no movement

Pause SLAM properly
```bash
ros2 service call /slam_toolbox/pause std_srvs/srv/Empty
```
Resume later
```bash
ros2 service call /slam_toolbox/resume std_srvs/srv/Empty
```


1. ✔ map → odom stays published
2. ✔ Map stable
3. ✔ Robot stable

