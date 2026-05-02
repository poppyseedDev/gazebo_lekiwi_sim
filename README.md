# gazebo_lekiwi_sim

Gazebo Harmonic simulation of a **LeKiwi mobile manipulator** — the SIGRobotics-UIUC 3-wheel omni base, with a 2D LD06 LiDAR — built on top of [Aditya Kamath's `lekiwi_ros2`](https://github.com/adityakamath/lekiwi_ros2).

See [MISSION.md](MISSION.md) for the broader project goals (mobile manipulation, language-driven tasks, sim-to-real). This repo covers **Phase 1 — Navigation**.

## Status

**Phase 1: complete.** Verified end-to-end in Gazebo Harmonic on Jazzy:
- Holonomic drive via `cmd_vel` (Gazebo `VelocityControl` plugin)
- 2D LiDAR mapping via `slam_toolbox`
- Autonomous frontier exploration via `explore_lite` + Nav2
- Saved-map navigation: AMCL localizes, Nav2 plans + drives to clicked goals (MPPI controller in Omni mode)
- Tested in three worlds: `empty.sdf`, `depot.sdf`, `small_house.sdf` (a 3-room apartment)

A reference saved map of the apartment is in [`src/lekiwi_ros2/lekiwi_navigation/maps/small_house.{pgm,yaml}`](src/lekiwi_ros2/lekiwi_navigation/maps/).

## Hardware modeled

- **Base**: LeKiwi — 3 omni wheels at 60° / 180° / 300°, kiwi-drive (holonomic)
- **Sensor**: 2D LiDAR (LD06) mounted ~16 cm above the floor
- **Pantilt + camera**: present in `lekiwi_description` for the real robot, **excluded from the sim URDF** (the tilt mount sits 5 cm above the laser plane and the LD06 would otherwise see it as an obstacle in front of the robot — see comment in `lekiwi.sim.urdf.xacro`).

## Requirements

- Ubuntu 24.04 (tested on ARM64 inside UTM/Apple Silicon and on x86_64)
- ROS 2 Jazzy desktop
- `ros-jazzy-ros-gz` (Gazebo Harmonic + bridge)
- `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`, `ros-jazzy-slam-toolbox`
- `ros-jazzy-teleop-twist-keyboard` (for manual driving)

## One-time setup

```bash
git clone git@github.com:poppyseedDev/gazebo_lekiwi_sim.git ~/lekiwi_ws
cd ~/lekiwi_ws/src

# Frontier explorer for active SLAM (upstream, vendored separately):
git clone https://github.com/robo-friends/m-explore-ros2

# (Real robot only — skip for sim-only work)
# git clone https://github.com/adityakamath/sts_hardware_interface

cd ~/lekiwi_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
echo "source $HOME/lekiwi_ws/install/setup.bash" >> ~/.bashrc
source install/setup.bash
```

## Reproducing Phase 1

The reliable flow uses **separate terminals** for each subsystem so DDS/lifecycle races don't sabotage startup. Each terminal needs the workspace overlay sourced (`~/.bashrc` does this automatically after the setup above).

### A. View the URDF

```bash
ros2 launch lekiwi_description display.launch.py
```

### B. Map an environment with autonomous exploration

**Terminal 1 — sim:**
```bash
ros2 launch lekiwi_gazebo sim_bringup.launch.py world:=small_house.sdf
```
Wait until Gazebo and RViz are both up.

**Terminal 2 — slam_toolbox:**
```bash
ros2 launch lekiwi_gazebo slam.launch.py
```
Verify it activated:
```bash
ros2 lifecycle get /slam_toolbox        # → active [3]
ros2 topic hz /map                      # → ~0.2 Hz
```

**Terminal 3 — Nav2 (no localization, slam supplies map):**
```bash
ros2 launch lekiwi_navigation nav2.launch.py \
  use_localization:=False use_composition:=False
```
Verify the BT navigator activated:
```bash
ros2 lifecycle get /bt_navigator        # → active [3]
```

**Terminal 4 — frontier explorer:**
```bash
ros2 run explore_lite explore --ros-args \
  --params-file ~/lekiwi_ws/install/lekiwi_gazebo/share/lekiwi_gazebo/config/explore_lite.yaml \
  -p use_sim_time:=true \
  -r /tf:=tf -r /tf_static:=tf_static
```
The robot will drive a coverage tour of the world. When `explore_lite` logs **"No frontiers found, stopping"**, exploration is done.

**Terminal 5 — save the map:**
```bash
ros2 run nav2_map_server map_saver_cli \
  -f $HOME/lekiwi_ws/src/lekiwi_ros2/lekiwi_navigation/maps/small_house \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=15.0
```

### C. Autonomous navigation against a saved map

**Terminal 1 — sim:**
```bash
ros2 launch lekiwi_gazebo sim_bringup.launch.py world:=small_house.sdf
```

**Terminal 2 — Nav2 with AMCL:**
```bash
ros2 launch lekiwi_navigation nav2.launch.py \
  map:=$HOME/lekiwi_ws/src/lekiwi_ros2/lekiwi_navigation/maps/small_house.yaml \
  use_composition:=False
```
AMCL auto-initializes at the spawn pose `(-1.5, 0, 0)` (configured via `amcl.set_initial_pose` in [`nav2_params.yaml`](src/lekiwi_ros2/lekiwi_navigation/config/nav2_params.yaml)).

**Terminal 3 — RViz with Nav2's prebuilt config (has the toolbar buttons):**
```bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz \
  --ros-args -p use_sim_time:=true
```

In RViz toolbar, click **Nav2 Goal** and click anywhere on the map. The robot plans a path (green line) and drives there.

CLI alternative for sending goals (no RViz click needed):
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}'
```

## Worlds available

| File | Use case |
|---|---|
| `small_house.sdf` | 3-room apartment with sofa, bed, kitchen counter — default SLAM/Nav2 demo |
| `empty.sdf` | Open ground with a few boxes — quick sanity tests |
| `depot.sdf` | Fuel-backed warehouse — needs internet on first run |

To use a different world: pass `world:=<file>` to `sim_bringup.launch.py`. Spawn at custom coords with `x:= y:=`.

## Package layout

| Package | Role | Sim/Real |
|---|---|---|
| `lekiwi_description` | URDF, meshes, RViz display launch | shared |
| `lekiwi_gazebo` | sim-only URDF overlay, worlds, ros_gz bridge, sim/slam launches, explore_lite config | sim |
| `lekiwi_navigation` | Nav2 params (omni-tuned), saved maps, Nav2 launch, EKF for real robot | shared |
| `lekiwi_control` | ros2_control config (STS3215 servos) | real |
| `lekiwi_bringup` | real-robot orchestration (incl. an active_slam launch template) | real |

## Tuning notes

- **OGRE1, not OGRE2** — UTM/Apple Silicon's GPU passthrough doesn't supply hardware EGL, so OGRE2 falls back to software rendering and the RTF crashes to ~5%. The launch defaults to `render_engine:=ogre`. On a real GPU, pass `render_engine:=ogre2` for nicer visuals.
- **PlanarMove plugin is gone in Harmonic** — this repo uses `VelocityControl` + `OdometryPublisher` (drop-in replacement, also in the `lekiwi.gazebo.xacro` overlay).
- **Pantilt removed in sim** — the camera mount sits 5 cm above the LD06 plane and would otherwise show up as a permanent "wall" in front of the robot.
- **Inflation 0.25 m** — sized for the 1.6 m hallway in `small_house`. Larger inflation makes the planner unable to find paths in narrow passages.
- **Holonomic Nav2** — `motion_model: Omni` for MPPI, `min_y_velocity_threshold: 0.001` (was 0.5 — a diff-drive default that silently kills lateral motion), `velocity_smoother` Y components ±0.4 (was 0).
- **slam_toolbox startup race** — slam can drop the first scan if it activates before the bridge has `/scan` flowing. The "Phase 1 reproduction" flow above starts slam in a separate terminal *after* the sim is up to avoid this. We tried an all-in-one `active_slam.launch.py` but it had unfixable DDS startup races on slow VMs and was removed; the multi-terminal flow is the supported path.
- **Killing stale sims**: `Ctrl+C` doesn't always reap children. If a relaunch shows ghost robots or wrong worlds, run `pkill -9 -f "gz sim|ruby.*gz|robot_state_publisher|parameter_bridge|slam_toolbox|rviz2|ros2 launch|explore"`.

## What's next (Phases 2+)

See [MISSION.md](MISSION.md). Short version:
- **Phase 2** — Camera-based object detection and semantic goals on top of the Nav2 stack
- **Phase 3** — Real-robot validation (Pi 5 + LD06 + STS3215 + camera, no arm)
- **Phase 4** — SO-101 arm integration + teleop manipulation
- **Phase 5** — LLM natural-language interface
- **Phase 6** — Full sim-to-real

## License

Apache-2.0.
