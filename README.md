# gazebo_lekiwi_sim

Gazebo Harmonic simulation of a **LeKiwi mobile manipulator** — the SIGRobotics-UIUC 3-wheel omni base, with a 2D LD06 LiDAR — built on top of [Aditya Kamath's `lekiwi_ros2`](https://github.com/adityakamath/lekiwi_ros2).

See [MISSION.md](MISSION.md) for the broader project goals (mobile manipulation, language-driven tasks, sim-to-real).

**Docs:**
- [docs/SETUP.md](docs/SETUP.md) — one-time install on a fresh machine
- [docs/USAGE.md](docs/USAGE.md) — command recipes (mapping, navigation, "go to ball")
- [docs/PROGRESS.md](docs/PROGRESS.md) — what's done so far, phase by phase

## Status

- **Phase 1 — Navigation: complete** (`91ed5c0`) — autonomous SLAM + saved-map navigation working end-to-end.
- **Phase 2 — Perception, v1: working** (`c351ed0`) — `"go to the red/yellow/blue ball"` demo. Sim camera + YOLOv8 + color-aware goal sender + Nav2.

Full per-phase status in [docs/PROGRESS.md](docs/PROGRESS.md). Reference saved map of the apartment is at [`src/lekiwi_ros2/lekiwi_navigation/maps/small_house.{pgm,yaml}`](src/lekiwi_ros2/lekiwi_navigation/maps/).

## Hardware modeled

- **Base**: LeKiwi — 3 omni wheels at 60° / 180° / 300°, kiwi-drive (holonomic)
- **Sensor**: 2D LiDAR (LD06) mounted ~16 cm above the floor
- **Pantilt + camera**: present in `lekiwi_description` for the real robot, **excluded from the sim URDF** (the tilt mount sits 5 cm above the laser plane and the LD06 would otherwise see it as an obstacle in front of the robot — see comment in `lekiwi.sim.urdf.xacro`).

## Quick start

Install + build per [docs/SETUP.md](docs/SETUP.md), then jump to [docs/USAGE.md](docs/USAGE.md) for the three core recipes:
- **A** — autonomous mapping (slam_toolbox + Nav2 + explore_lite)
- **B** — saved-map navigation (Nav2 + AMCL + Nav2-Goal clicks)
- **C** — "go to the red ball" (sim camera + YOLO + color goal-sender)

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
