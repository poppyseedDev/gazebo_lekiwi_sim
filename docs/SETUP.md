# One-time setup on a fresh machine

For the LeKiwi sim stack on Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic.

## Apt prerequisites

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-joint-state-publisher-gui \
  python3-colcon-common-extensions python3-rosdep
```

If `pip` isn't installed (Ubuntu 24.04 ships without it):
```bash
curl -sS https://bootstrap.pypa.io/get-pip.py | python3 - --user --break-system-packages
```

## Clone + vendored dependencies

```bash
git clone git@github.com:poppyseedDev/gazebo_lekiwi_sim.git ~/lekiwi_ws
cd ~/lekiwi_ws/src

# Frontier explorer for active SLAM (Phase 1)
git clone https://github.com/robo-friends/m-explore-ros2

# YOLO ROS 2 wrapper (Phase 2)
git clone https://github.com/mgonzs13/yolo_ros

# Real-robot only (skip for sim work; see lekiwi_bringup/)
# git clone https://github.com/adityakamath/sts_hardware_interface
```

## Python deps for YOLO

```bash
~/.local/bin/pip install --user --break-system-packages \
  "ultralytics==8.4.6" "lap" "numpy<2"
```

(Note: this also pulls torch + a CUDA toolkit; on a CPU-only machine those are unused but installed.)

## Build the workspace

```bash
cd ~/lekiwi_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
```

## Auto-source the overlay in new shells

```bash
echo "source $HOME/lekiwi_ws/install/setup.bash" >> ~/.bashrc
source install/setup.bash
```

After this you can jump to [USAGE.md](USAGE.md) for command recipes.

## Sim performance notes

- **OGRE1, not OGRE2** — `sim_bringup.launch.py` defaults to `render_engine:=ogre`. On UTM/Apple Silicon the OGRE2 path falls back to software EGL and tanks the RTF; OGRE1 uses GLX and works. On a real GPU pass `render_engine:=ogre2` for nicer visuals.
- **Pantilt module is excluded from the sim URDF** — its camera mount sits 5 cm above the LD06 plane; with it attached the lidar would see a permanent obstacle directly in front.
- **CPU is the bottleneck** in UTM/VM. Closing RViz when not needed saves ~1.5 cores. Bumping the VM CPU count from 6 → 8+ in UTM Settings helps everything.
- **No GPU acceleration in UTM** — UTM doesn't expose Metal as CUDA, and PyTorch's MPS backend only works on macOS host directly. YOLO and Gazebo sensor rendering are CPU-only here. See README "Tuning notes" for context and migration paths.
