# TurtleBot3 WanderBot

A simple ROS Noetic wanderbot implementation for TurtleBot3 robots that performs random walk with obstacle avoidance.

## Overview

WanderBot is a reactive navigation node that makes the TurtleBot3 wander autonomously by:
- Moving forward with random angular jitter
- Detecting obstacles using laser scan data
- Rotating randomly when obstacles are detected
- Resuming forward motion after clearing obstacles

## Features

- **Simple State Machine**: Two states (MOVE_FORWARD, ROTATE)
- **Obstacle Avoidance**: Uses front sector of laser scan for detection
- **Random Behavior**: Randomized rotation direction and duration
- **Configurable Parameters**: Tunable speeds, distances, and angles
- **Model Support**: Optimized parameters for both Burger and Waffle models

## Prerequisites

- ROS Noetic
- Python 3
- TurtleBot3 packages
- TurtleBot3 Gazebo simulator (for simulation)

```bash
sudo apt update
sudo apt install -y ros-noetic-turtlebot3-gazebo
```

## Installation

### 1. Create or Navigate to Your Catkin Workspace

```bash
cd ~/catkin_ws/src
```

### 2. Create Package (if new)

```bash
catkin_create_pkg turtle_wander rospy std_msgs sensor_msgs geometry_msgs
```

### 3. Add the WanderBot Script

Create `~/catkin_ws/src/turtle_wander/scripts/wanderbot.py` with the provided Python code and make it executable:

```bash
chmod +x ~/catkin_ws/src/turtle_wander/scripts/wanderbot.py
```

### 4. Create Launch Directory and Files

```bash
mkdir -p ~/catkin_ws/src/turtle_wander/launch
```

Add the launch files (see Launch Files section below).

### 5. Build the Workspace

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

### Running with Waffle Model (Recommended)

**Terminal A - Start Gazebo Simulation:**
```bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

**Terminal B - Run WanderBot:**
```bash
source ~/catkin_ws/devel/setup.bash
rosrun turtle_wander wanderbot.py
```

**Or use the combined launch file:**
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch turtle_wander wanderbot_waffle.launch
```

### Running with Burger Model

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtle_wander wanderbot_waffle.launch model:=burger
```

Or adjust parameters manually:
```bash
rosrun turtle_wander wanderbot.py _forward_speed:=0.18 _min_obstacle_dist:=0.45
```

## Launch Files

### wanderbot_waffle.launch

Create `launch/wanderbot_waffle.launch`:

```xml
<launch>
  <!-- default to waffle model -->
  <arg name="model" default="waffle"/>

  <!-- export the TURTLEBOT3_MODEL so included launch sees it -->
  <env name="TURTLEBOT3_MODEL" value="$(arg model)"/>

  <!-- Use standard turtlebot3 gazebo world (this will spawn the waffle model) -->
  <include file="$(find turtlebot3_gazebo)/launch/turtlebot3_world.launch">
    <arg name="gui" value="true"/>
    <arg name="use_sim_time" value="true"/>
  </include>

  <!-- Start the wanderbot node in your package -->
  <node pkg="turtle_wander" type="wanderbot.py" name="wanderbot" output="screen">
    <!-- tuned parameters for waffle (bigger robot -> more cautious) -->
    <param name="~forward_speed" value="0.15"/>
    <param name="~min_obstacle_dist" value="0.55"/>
    <param name="~rotate_speed" value="0.6"/>
    <param name="~front_angle_deg" value="35"/>
    <param name="~rate_hz" value="10"/>
  </node>
</launch>
```

## Parameters

### Waffle Model (Recommended Settings)

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `forward_speed` | 0.15 | 0.12-0.18 | Linear velocity in m/s |
| `min_obstacle_dist` | 0.55 | 0.5-0.7 | Obstacle detection threshold in meters |
| `rotate_speed` | 0.6 | 0.4-0.8 | Angular velocity during rotation in rad/s |
| `front_angle_deg` | 35 | 30-40 | Front sector angle for obstacle detection |
| `jitter_angle_rad` | 0.15 | 0.1-0.2 | Random angular jitter while moving forward |
| `rotate_time_min` | 0.6 | 0.4-1.0 | Minimum rotation duration in seconds |
| `rotate_time_max` | 1.4 | 1.0-2.0 | Maximum rotation duration in seconds |
| `rate_hz` | 10 | 5-20 | Control loop frequency in Hz |

### Burger Model Settings

For the smaller Burger model, use these adjusted parameters:

```bash
rosrun turtle_wander wanderbot.py \
  _forward_speed:=0.18 \
  _min_obstacle_dist:=0.45 \
  _rotate_speed:=0.6 \
  _front_angle_deg:=30
```

## Tuning Tips

- **Robot pushes into obstacles**: Increase `min_obstacle_dist` or reduce `forward_speed`
- **Robot rotates too often**: Reduce `min_obstacle_dist` or narrow `front_angle_deg`
- **Robot gets stuck**: Increase `rotate_time_min/max` or add backup behavior
- **Motion too straight**: Increase `jitter_angle_rad`
- **Response too slow**: Increase `rate_hz`

## Topics

### Subscribed
- `/scan` (sensor_msgs/LaserScan): Laser scan data for obstacle detection

### Published
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands to move the robot

## Architecture

### State Machine

```
MOVE_FORWARD ──[obstacle detected]──> ROTATE
     ^                                   |
     └────────[rotation complete]────────┘
```

### Behavior Logic

1. **MOVE_FORWARD State**:
   - Check front sector of laser scan
   - If obstacle closer than `min_obstacle_dist`: transition to ROTATE
   - Otherwise: move forward with random angular jitter

2. **ROTATE State**:
   - Rotate in random direction (left or right)
   - Continue for random duration between `rotate_time_min` and `rotate_time_max`
   - After rotation complete: transition to MOVE_FORWARD

## Safety Notes

⚠️ **Important Safety Considerations**:

- Test in simulation first before using on real hardware
- Use low speeds initially and gradually increase
- Always have an emergency stop (e-stop) available for real robots
- Monitor the robot during operation
- This is a reactive behavior with no mapping or localization
- Not suitable for complex environments or mission-critical applications

## Troubleshooting

### Robot doesn't move
- Check if `/scan` topic is publishing: `rostopic echo /scan`
- Verify `/cmd_vel` topic exists: `rostopic list | grep cmd_vel`
- Check node is running: `rosnode list | grep wanderbot`

### Robot spins in place
- Reduce `rotate_speed`
- Check `forward_speed` is not zero
- Verify laser scan is working properly

### Topics have different names
Remap topics in launch file or command line:
```bash
rosrun turtle_wander wanderbot.py \
  /scan:=/your_scan_topic \
  /cmd_vel:=/your_cmd_vel_topic
```

### Permission denied error
```bash
chmod +x ~/catkin_ws/src/turtle_wander/scripts/wanderbot.py
```

## Future Enhancements

Potential improvements to consider:

- Add backup behavior when stuck
- Implement odometry-based rotation control
- Add wall-following mode
- Include mapping integration
- Add dynamic parameter reconfiguration
- Implement more sophisticated state machine
- Add multi-robot support

## License

This code is provided as-is for educational and research purposes.

## Contributing

Feel free to submit issues, fork the repository, and create pull requests for any improvements.

## Author

Created for ROS Noetic with TurtleBot3 support.

