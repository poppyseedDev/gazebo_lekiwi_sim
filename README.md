# gazebo_lekiwi_sim

Gazebo Harmonic simulation of a **LeKiwi mobile manipulator** — the SIGRobotics-UIUC 3-wheel omni base from the LeRobot/Hugging Face ecosystem, with a 2D LD06 LiDAR and an SO-101 arm mount. Built on top of [Aditya Kamath's `lekiwi_ros2`](https://github.com/adityakamath/lekiwi_ros2); the simulation pieces here are the new contribution.

## Hardware modeled

- **Base**: LeKiwi — 3 omni wheels at 60° / 180° / 300°, kiwi-drive (holonomic)
- **Sensor**: 2D LiDAR (LD06) mounted ~16 cm above the floor
- **Arm**: SO-101 mount as static visual (no manipulation simulation)

## Requirements

- Ubuntu 24.04 (tested on ARM64 inside UTM/Apple Silicon and on x86_64)
- ROS 2 Jazzy desktop
- `ros-jazzy-ros-gz` (Gazebo Harmonic + bridge)
- `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`, `ros-jazzy-slam-toolbox`
- `ros-jazzy-teleop-twist-keyboard` (for driving)
- Optional: `ros-jazzy-joint-state-publisher-gui` (joint sliders in `display.launch.py`)

## Build

```bash
cd ~/lekiwi_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> Add `source ~/lekiwi_ws/install/setup.bash` to your `~/.bashrc` so future shells pick up the overlay.

## Quick start

Each command below runs in its own terminal (all need the workspace overlay sourced).

**1. View the URDF in RViz**
```bash
ros2 launch lekiwi_description display.launch.py
```

**2. Run the sim**
```bash
ros2 launch lekiwi_gazebo sim_bringup.launch.py world:=small_house.sdf
```
Available worlds: `small_house.sdf` (3-room apartment, default for SLAM), `empty.sdf` (open with a few boxes), `depot.sdf` (Fuel-backed depot — needs internet on first run).

**3. Drive it**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
The robot is holonomic — `u/o`, `m/.` strafe diagonally; `j/l` rotate.

**4. SLAM in parallel**
```bash
ros2 launch lekiwi_gazebo slam.launch.py
```
Add a **Map** display in RViz (Add → By topic → /map → Map) to watch coverage build up.

**5. Save the map**
```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/lekiwi_ws/src/lekiwi_ros2/lekiwi_navigation/maps/<name> \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=15.0
```

**6. Navigate against a saved map**
```bash
ros2 launch lekiwi_navigation nav2.launch.py \
  map:=$HOME/lekiwi_ws/src/lekiwi_ros2/lekiwi_navigation/maps/<name>.yaml
```
Use **2D Pose Estimate** in RViz to localise, then **Nav2 Goal** to send goals.

## Auto-mapping

A scripted exploration tour for the bundled `small_house.sdf` is included:

```bash
# T1
ros2 launch lekiwi_gazebo sim_bringup.launch.py world:=small_house.sdf
# T2
ros2 launch lekiwi_gazebo slam.launch.py
# T3
ros2 run lekiwi_gazebo auto_map.py
```

It drives a 19-waypoint tour through all three rooms (passing the doorways twice for loop closure), then saves the map to `lekiwi_navigation/maps/small_house.{pgm,yaml}`. Stuck waypoints are skipped after 4 s of zero progress.

## Package layout

| Package | Role | Sim/Real |
|---|---|---|
| `lekiwi_description` | URDF, meshes, RViz display launch | shared |
| `lekiwi_gazebo` | sim-only URDF overlay, worlds, ros_gz bridge, sim/slam/teleop launches, auto_map | sim |
| `lekiwi_navigation` | EKF (real), Nav2 params (omni-tuned), saved maps, Nav2 launch | shared |
| `lekiwi_control` | ros2_control config (STS3215 servos) | real |
| `lekiwi_bringup` | real-robot orchestration | real |

The sim overlay (`lekiwi_gazebo/urdf/lekiwi.sim.urdf.xacro`) intentionally **skips** the real-robot `lekiwi.control.xacro` so the sim doesn't try to load the STS / BNO055 hardware plugins. Sim drive is via Gazebo's `VelocityControl` plugin (holonomic), odom via `OdometryPublisher`.

## Tuning notes

- **OGRE1, not OGRE2** — UTM/Apple Silicon's GPU passthrough doesn't give EGL, so OGRE2 falls back to software rendering and tanks the RTF. The launch defaults to `render_engine:=ogre`. On a real GPU, pass `render_engine:=ogre2` for nicer visuals.
- **dartsim mesh collisions** are disabled (warnings about `geometry … couldn't be created`). The robot stays grounded thanks to dartsim's default fallback. For accurate contact physics, replace mesh `<collision>` elements in `lekiwi_description/urdf/base/base.module.xacro` with primitives.
- **PlanarMove plugin is gone in Harmonic** — use `VelocityControl` + `OdometryPublisher` (which is what this repo does).
- **Nav2 omni tuning**: `motion_model: Omni` for MPPI, `PreferForwardCritic` disabled, `vy_max: 0.4`, `robot_radius: 0.18`, `robot_base_frame: base_footprint`. Jazzy migrations applied (`error_code_name_prefixes`).
- **Killing stale sims**: `Ctrl+C` on `ros2 launch` doesn't always reap children. If a relaunch shows the wrong world or "ghost" robots, run `pkill -9 -f "gz sim|ruby.*gz|robot_state_publisher|parameter_bridge|slam_toolbox|rviz2|ros2 launch"` first.

## Roadmap

- **Phase E**: real-robot deployment on the Pi (separate bringup launch wired to the LD06 driver and the STS3215 driver in [`sts_hardware_interface`](https://github.com/adityakamath/sts_hardware_interface), which is **not** included in this repo — clone it into `src/` when targeting hardware).
- **Phase F**: SO-101 arm as static include from [`TheRobotStudio/SO-ARM100`](https://github.com/TheRobotStudio/SO-ARM100).
- Per-wheel rotation in sim via `gz_ros2_control` + `omni_wheel_drive_controller` (currently the wheels slide; visual-only).

## License

Apache-2.0.
