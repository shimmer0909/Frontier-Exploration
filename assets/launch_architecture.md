## 1️⃣ Why this launch structure exists

Simulation, navigation, and exploration are intentionally decoupled to:
- avoid lifecycle conflicts
- keep TF ownership clear
- allow partial system bringup

## 2️⃣ Why Frontier Explorer does NOT own Nav2

The frontier_exploration package does not start Nav2 directly.
Instead, Nav2 is composed at the system level to avoid:
- hidden controller activation
- TF duplication
- recovery behavior side effects

## 3️⃣ Explanation of each launch file

### simulation.launch.py
1. **Purpose:**
Brings up the complete simulation environment and robot state without any navigation or exploration logic.
2. **Starts:**
1. Gazebo world and TurtleBot3 model
2. robot_state_publisher (URDF → TF)
3. Fake odometry publisher (/odom)
3. **Why this exists:**
1. Simulation is often needed without Nav2 or SLAM
2. Keeps TF ownership clear (odom → base_*)
3. Allows visual and TF debugging before navigation is introduced
4. **When to use it alone:**
1. Verifying TF tree
2. Checking sensor data (LaserScan, depth, etc.)
3. Debugging robot description / URDF
4. Debugging odometry issues
```bash
ros2 launch frontier_exploration simulation.launch.py
```

### navigation.launch.py
1. **Purpose:**
Brings up the navigation stack and mapping while assuming a robot (real or simulated) is already publishing TF and odometry.
2. **Starts:**
1. slam_toolbox (online mapping)
2. Nav2 bringup (planner, controller, behavior tree)
3. RViz (Nav2-compatible config)
3. **Does NOT start:**
1. Gazebo
2. Robot description
3. Odometry publishers
4. **Why this exists:**
1. Nav2 and SLAM have complex lifecycles and recovery behaviors
2. Keeping them isolated avoids accidental motion during debugging
3. Makes it easy to pause, resume, or replace SLAM independently
5. **When to use it alone:**
1. Debugging SLAM stability
2. Debugging Nav2 costmaps
3. Tuning Nav2 parameters
4. Investigating recovery behaviors
```bash
ros2 launch frontier_exploration navigation.launch.py
```

### frontier_explorer.launch.py
1. **Purpose:**
Runs only the frontier exploration logic, independent of simulation or navigation setup.
2. **Starts:**
1. map_listener
2. frontier_detector
3. frontier_selector
3. **Assumes:**
1. /map is being published (from SLAM)
2. Nav2 is active and accepting goals
3. Valid TF tree exists
4. **Why this exists:**
1. Exploration logic should not depend on how the robot is simulated
2. Allows reusabiity
3. Makes unit testing and debugging easier
5. **When to use it alone:**
1. Testing frontier detection correctness
2. Visualizing frontier points
3. Debugging clustering and cost functions
4. Validating goal selection logic
```bash
ros2 launch frontier_exploration frontier_explorer.launch.py
```

### full_exploration.launch.py
1. **Purpose:**
Provides a one-command entry point that composes the full autonomous exploration system.
2. **Includes:**
1. simulation.launch.py
2. navigation.launch.py
3. frontier_explorer.launch.py
3. **Why this exists:**
1. Reduces user friction
2. Hides launch complexity without hiding system behavior
3. Ideal for demos, testing, and first-time users
4. **This is the recommended way to run the project.**
```bash
ros2 launch frontier_exploration full_exploration.launch.py
```
5. **When to use it:**
1. 'full_exploration.launch.py' is intended for demos and end-to-end testing.
2. During development or debugging, it is recommended to launch components
3. Individually to avoid cascading failures.

## Real Robot Usage

For real robots:
- Use `navigation.launch.py` + `frontier_explorer.launch.py`
- Do NOT use `simulation.launch.py`
- Ensure odometry and TF are published by hardware drivers

The exploration logic is hardware-agnostic.

## 4️⃣ Manual commands lauch for debugging

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
