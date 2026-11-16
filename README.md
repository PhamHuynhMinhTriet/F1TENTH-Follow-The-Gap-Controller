# F1TENTH Reactive Controller: Follow-the-Gap + Disparity Extender

**Track:** Catalunya  
**Platform:** F1TENTH Gym Simulator  
**Framework:** ROS 2 Humble  
**Language:** Python

## 1. Overview

This project implements a reactive autonomous racing controller for the F1TENTH Gym Simulator using the Catalunya map.

The controller combines the Follow-the-Gap algorithm with a Disparity Extender to improve obstacle avoidance and driving stability.

It uses LiDAR data to calculate steering and speed commands, while odometry is used for lap detection and lap tracking.

The controller also includes lap-based speed management and local slow-down zones for previously problematic track locations.

## 2. Control Algorithm

### 2.1 Follow-the-Gap and Disparity Extender

The controller processes every second LiDAR scan.

The main steps are:

1. **LiDAR preprocessing:** Replace invalid measurements and clip ranges to the maximum detection distance.
2. **Disparity detection:** Detect significant differences between adjacent LiDAR beams.
3. **Disparity extension:** Expand obstacle boundaries according to the vehicle's half-width.
4. **Scan smoothing:** Apply a moving average to reduce sudden measurement changes.
5. **Gap selection:** Identify the most suitable open direction, prioritizing directions close to straight ahead.
6. **Steering dead-band:** Ignore very small steering angles to reduce unnecessary movement on straight sections.
7. **Lane centering:** Apply a correction based on the left and right wall distances.
8. **Steering control:** Calculate and limit the final steering angle.
9. **Adaptive smoothing:** Adjust steering response depending on the driving direction.

### 2.2 Speed Control

The controller calculates speed using three main limits:

- `v_curve`: Speed limit based on steering angle.
- `v_brake`: Braking limit based on forward obstacle distance.
- `vlim`: Local speed limit based on the current track zone.

The braking equation is:

\[
v_{brake}=\sqrt{2 \cdot decel \cdot \max(d_{front}-d_{margin},0)}
\]

The final speed is calculated using the controller's configured minimum and maximum speed constraints.

### 2.3 Automatic Speed Management

The controller uses odometry to detect completed laps.

Lap detection requires the vehicle to:
- Move farther than `lap_leave` from its starting position.
- Return within `lap_return` of the starting position.

After each lap, the controller evaluates the minimum lateral clearance.

**Clean lap:**
- Increase the maximum speed by `vstep`, up to `vmax_max`.

**Dirty lap:**
- Create a local slow-down zone around the location with the smallest wall clearance.
- Retry the lap with a reduced local speed limit.
- After repeated unsuccessful attempts, restore the best known speed and lock the learned setting.

Lap times and learned parameters are saved for future use.

## 3. Code Structure

| File | Description |
|---|---|
| `fast_gap_catalunya.py` | Main controller and ROS 2 node |
| `maps/Catalunya_map.png` | Catalunya occupancy map image |
| `maps/Catalunya_map.yaml` | Catalunya map configuration |
| `catalunya_laps.csv` | Lap time and performance records |
| `catalunya_state.json` | Saved speed-learning state |

### Main Classes and Functions

**FastGapCore**

Contains the main controller logic.

- `step()` – Processes LiDAR data and calculates steering and speed.
- `lateral_sides()` – Measures left and right wall distances.
- `side_clearance()` – Evaluates lateral clearance.
- `lap_done()` – Handles lap completion and speed management.
- `zone_factor()` – Calculates local speed limits.
- `save()` – Saves the learned controller state.

**FastGap(Node)**

Connects the controller to ROS 2.

- Subscribes to `/scan`.
- Subscribes to `/ego_racecar/odom`.
- Publishes driving commands to `/drive`.

## 4. ROS 2 Communication

| Topic | Message Type | Purpose |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR measurements |
| `/ego_racecar/odom` | `nav_msgs/Odometry` | Vehicle position |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | Steering and speed commands |

## 5. Requirements

- Ubuntu 22.04
- ROS 2 Humble
- F1TENTH Gym ROS bridge
- Python 3
- NumPy
- Ackermann messages

## 6. Main Parameters

| Parameter | Default | Description |
|---|---:|---|
| `vmax` | 50.0 | Maximum target speed |
| `vmin` | 50.0 | Minimum speed setting |
| `decel` | 7.0 | Deceleration parameter |
| `dmargin` | 1.0 | Safety distance margin |
| `vstep` | 0.25 | Speed adjustment step |
| `vmax_max` | 50.0 | Maximum allowed speed |
| `half_width` | 0.25 | Vehicle half-width and safety margin |
| `disp_thresh` | 0.5 | Disparity detection threshold |
| `kp` | 1.0 | Steering gain |
| `kc` | 0.4 | Lane-centering gain |
| `skip` | 2 | Process every second LiDAR scan |
| `load` | false | Load saved controller state |
| `freeze` | false | Lock learned speed and disable further learning |

Additional parameters can be configured through ROS 2 command-line arguments.

## 7. Installation and Execution

### Clone the Repository

```bash
git clone https://github.com/PhamHuynhMinhTriet/F1TENTH-Follow-The-Gap-Controller.git
cd F1TENTH-Follow-The-Gap-Controller
```

### Configure the Catalunya Map

Edit the simulator configuration file:

```bash
gedit ~/sim_ws/src/f1tenth_gym_ros/config/sim.yaml
```

Set the map path to the actual absolute path of the Catalunya map:

```yaml
map_path: '/absolute/path/to/Catalunya_map'
map_img_ext: '.png'
```

### Build and Launch the Simulator

In Terminal 1:

```bash
cd ~/sim_ws
colcon build
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### Run the Controller

In Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/sim_ws/install/setup.bash

cd ~/F1TENTH-Follow-The-Gap-Controller
```

Run the controller:

```bash
python3 fast_gap_catalunya.py
```

The package name, executable and file paths should match the actual project installation.

## 8. Competition Objectives

**Challenge 1 – Safe Driving**

Complete 10 consecutive laps without collisions.

**Challenge 2 – Fastest Lap**

Achieve the fastest valid lap time on the Catalunya track.

## 9. Results

| Mode | Maximum Speed (m/s) | Best Lap Time (s) | Notes |
|---|---:|---:|---|
| Testing | TBD | TBD | Results to be recorded |
| Competition | TBD | TBD | Results to be recorded |

Actual results will be added after simulation testing.

## 10. Video Demonstration

The demonstration video should show:

- The vehicle driving on the Catalunya track.
- The lap counter.
- The timer and individual lap times.
- The vehicle completing the required laps.

## 11. Author

**Project:** F1TENTH Reactive Controller  
**Algorithm:** Follow-the-Gap + Disparity Extender  
**Track:** Catalunya  
**Framework:** ROS 2 Humble
