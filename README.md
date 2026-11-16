# F1TENTH Reactive Controller: Follow-the-Gap + Disparity Extender

**Track:** Catalunya  
**Platform:** F1TENTH Gym Simulator  
**Framework:** ROS 2 Humble  
**Language:** Python

## 1. Project Overview

This project implements a reactive autonomous racing controller for the F1TENTH Gym Simulator using the Catalunya map.

The controller combines the Follow-the-Gap algorithm with a Disparity Extender to identify open driving directions and improve obstacle avoidance.

LiDAR data is used to calculate steering and speed commands, while odometry provides vehicle position information for lap detection and lap-time recording.

The controller also includes steering smoothing, lateral clearance monitoring, local speed-zone calculation, and CSV/JSON data logging.

## 2. Control Algorithm

### 2.1 Follow-the-Gap and Disparity Extender

The controller processes every second LiDAR scan by default.

The main processing steps are:

1. **LiDAR preprocessing:** Replace invalid measurements and limit ranges to the configured maximum detection distance.
2. **Field-of-view selection:** Process LiDAR measurements within the configured front field of view.
3. **Disparity detection:** Detect significant differences between adjacent LiDAR beams.
4. **Disparity extension:** Extend obstacle boundaries according to the vehicle half-width.
5. **Scan smoothing:** Apply a moving-average filter to reduce sudden measurement changes.
6. **Gap selection:** Select a beam from the open region, prioritizing directions close to straight ahead.
7. **Steering dead-band:** Ignore very small steering commands to reduce unnecessary steering changes.
8. **Lane centering:** Apply a correction based on the difference between left and right lateral distances.
9. **Steering control:** Apply steering gain and limit the final steering angle.
10. **Adaptive smoothing:** Adjust steering smoothing according to the steering demand.

### 2.2 Speed Control

The controller calculates speed using three limits:

- `v_curve`: Speed limit based on steering angle.
- `v_brake`: Braking limit based on forward obstacle distance.
- `vlim`: Local speed limit based on the current position and slow-down zones.

The braking equation is:

\[
v_{brake}=\sqrt{2 \cdot decel \cdot \max(d_{front}-d_{margin},0)}
\]

The final speed is calculated using the configured minimum-speed constraint and the three speed limits.

**Important configuration note:** The current configuration uses `vmax = 50.0` and `vmin = 50.0`. With these equal values, the minimum-speed constraint can prevent the braking calculation from reducing the commanded speed. The configured values are retained as requested, but simulator speed limits and braking behavior must be checked before testing.

### 2.3 Lap Detection and Speed Management

The controller uses odometry to detect completed laps.

Lap detection requires the vehicle to:

- Move farther than `lap_leave` from its starting position.
- Return within `lap_return` of the starting position.

After each detected lap, the controller:

- Calculates the lap time.
- Records the minimum lateral clearance observed during processed scans.
- Classifies the lap as clean or unclean using `clear_thr`.
- Saves lap information to a CSV file.

When automatic speed management is enabled, a clean lap increases the configured speed value by `vstep`, up to `vmax_max`.

For an unclean lap, the controller records a local slow-down zone and updates its stored speed value according to the configured limits.

**Current configuration:** Since `vmax`, `vmin`, and `vmax_max` are all 50.0, the automatic speed adjustment has no available speed-increase range. The current implementation also does not include a repeated-failure counter or an automatic learning lock.

## 3. Project Structure

```text
F1TENTH-Follow-The-Gap-Controller/
│
├── README.md
├── fast_gap_catalunya.py
│
└── maps/
    ├── Catalunya_map.png
    └── Catalunya_map.yaml
```

Additional files are generated when the controller runs:

| File | Description |
|---|---|
| `catalunya_laps.csv` | Lap times, minimum lateral clearance, speed and lap classification |
| `catalunya_state.json` | Saved speed configuration and slow-down zones |

## 4. Main Classes and Functions

### FastGapCore

Contains the main controller logic.

| Function | Description |
|---|---|
| `step()` | Processes LiDAR data and calculates steering and speed |
| `lateral_sides()` | Measures left and right lateral distances |
| `side_clearance()` | Evaluates minimum lateral clearance |
| `lap_done()` | Detects completed laps and records lap information |
| `zone_factor()` | Calculates local speed limits |
| `save()` | Saves controller state to a JSON file |
| `load()` | Loads previously saved controller state |

### FastGap(Node)

Connects the controller to ROS 2.

- Subscribes to `/scan`.
- Subscribes to `/ego_racecar/odom`.
- Publishes driving commands to `/drive`.
- Processes every second LiDAR scan by default.

## 5. ROS 2 Communication

| Topic | Message Type | Purpose |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR measurements |
| `/ego_racecar/odom` | `nav_msgs/Odometry` | Vehicle position |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | Steering and speed commands |

## 6. Requirements

- Ubuntu 22.04
- ROS 2 Humble
- F1TENTH Gym Simulator
- F1TENTH Gym ROS bridge
- Python 3
- NumPy
- `ackermann_msgs`
- `sensor_msgs`
- `nav_msgs`
- `rclpy`

## 7. Main Parameters

| Parameter | Default | Description |
|---|---:|---|
| `vmax` | 50.0 | Maximum target speed |
| `vmin` | 50.0 | Minimum speed constraint |
| `decel` | 7.0 | Deceleration parameter |
| `dmargin` | 1.0 | Forward safety distance margin |
| `vstep` | 0.25 | Speed adjustment step |
| `vmax_max` | 50.0 | Maximum allowed speed |
| `half_width` | 0.25 | Vehicle half-width for obstacle extension |
| `disp_thresh` | 0.5 | Disparity detection threshold |
| `kp` | 1.0 | Steering gain |
| `kc` | 0.4 | Lane-centering gain |
| `skip` | 2 | Process every second LiDAR scan |
| `max_range` | 10.0 | Maximum LiDAR range |
| `fov_deg` | 90.0 | Front field-of-view half-angle |
| `clear_thr` | 0.17 | Minimum lateral clearance threshold |
| `load` | false | Load saved controller state |
| `freeze` | false | Disable automatic speed adjustment |

Additional parameters can be configured through ROS 2 command-line arguments.

## 8. Installation and Execution

### 8.1 Clone the Repository

```bash
git clone https://github.com/PhamHuynhMinhTriet/F1TENTH-Follow-The-Gap-Controller.git

cd F1TENTH-Follow-The-Gap-Controller
```

### 8.2 Configure the Catalunya Map

Open the simulator configuration file:

```bash
gedit ~/sim_ws/src/f1tenth_gym_ros/config/sim.yaml
```

Set `map_path` to the absolute path of the map file without the `.png` extension.

For example, if the repository is cloned into your home directory:

```yaml
map_path: '/home/YOUR_USERNAME/F1TENTH-Follow-The-Gap-Controller/maps/Catalunya_map'
map_img_ext: '.png'
```

Replace `YOUR_USERNAME` with your actual Ubuntu username.

The PNG and YAML map files must be in the same directory and have matching base filenames.

### 8.3 Build and Launch the Simulator

In Terminal 1:

```bash
cd ~/sim_ws

colcon build

source install/setup.bash

ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### 8.4 Run the Controller

In Terminal 2:

```bash
source /opt/ros/humble/setup.bash

source ~/sim_ws/install/setup.bash

cd ~/F1TENTH-Follow-The-Gap-Controller

python3 fast_gap_catalunya.py
```

The simulator workspace, package installation and map path must match the actual local environment.

## 9. Competition Objectives

### Challenge 1 – Safe Driving

Complete 10 consecutive laps without collisions.

### Challenge 2 – Fastest Lap

Achieve the fastest valid lap time on the Catalunya track.

The controller must be tested in the simulator to verify collision-free driving, lap detection and timing accuracy.

## 10. Results

| Mode | Maximum Speed (m/s) | Best Lap Time (s) | Notes |
|---|---:|---:|---|
| Testing | TBD | TBD | Pending simulation |
| Competition | TBD | TBD | Pending competition |

Actual results will be added after simulation testing.

## 11. Video Demonstration

The following video demonstrates the F1TENTH autonomous racing controller and its operation.

**Simulation Video:** [Watch the F1TENTH Catalunya Demonstration](https://www.youtube.com/watch?v=gBrnQjDW3K0)

The video is provided as visual evidence of the project implementation and controller operation.

## 12. Author

**Project:** F1TENTH Reactive Controller  
**Algorithm:** Follow-the-Gap + Disparity Extender  
**Track:** Catalunya  
**Framework:** ROS 2 Humble  
**Language:** Python
