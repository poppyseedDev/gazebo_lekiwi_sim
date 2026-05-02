# Project progress

Status against [MISSION.md](../MISSION.md). Latest first.

---

## Phase 2 — Perception & Semantic Goals (in progress)

### v1: color-aware "go to the X ball" — DONE (commit `c351ed0`, 2026-05-02)

**End-to-end demo working**: command `/target_color: red|yellow|blue` → robot searches the world (spin + room-to-room), navigates to the matching ball, fine-adjusts to centre it in the camera view.

What's in place:
- **Sim camera** — `sim_camera_link` + `sim_camera_optical_frame` added to `lekiwi.gazebo.xacro`. Mounted at the real-robot height (~6.6 cm above floor), forward-facing, ROS optical-convention frame. 320×240 RGB at 15 Hz, 60° HFOV. Bridged via `ros_gz_bridge.yaml` to `/camera/image` + `/camera/camera_info`. Description's existing `camera_link` left untouched (its rotation is for the visual mesh, not for sensor mounting).
- **Three colored balls** in `small_house.sdf` — yellow, red, blue spheres on the living-room floor, all detected by YOLOv8 as COCO class `sports ball`.
- **YOLOv8 detector** — wrapped via `lekiwi_gazebo/launch/yolo.launch.py` around the upstream `yolo_ros` (mgonzs13). CPU defaults: yolov8n.pt, 192×192 inference, threshold 0.25. Runs at ~2-4 Hz on the VM.
- **Goal-sender brain** — `lekiwi_gazebo/scripts/go_to_color_ball.py`:
  - HSV-based color filter on bbox-center pixels
  - 2D→3D ground-plane projection via camera intrinsics + map ← optical TF
  - Search loop: 60° spin every 5 s, then drive to next room waypoint
  - Navigation lock: once a navigate-to-ball goal is published, search is suppressed until arrival or 60 s timeout (prevents mid-trip diversions)
  - Fine-adjust: P-controller yaw correction from pixel error, with three termination conditions (ball centered ±25 px, max 4 attempts, or 6 s with no detection = robot is so close YOLO can't classify it = "in front")
- **Trigger interface** — `ros2 topic pub --once /target_color std_msgs/String "{data: red}"`. Will become the LLM/voice command target in Phase 5.

Known limitations:
- Color filter is hand-tuned hue ranges; brittle to lighting changes.
- YOLO trained on real-photo COCO; flat-shaded sim primitives work for spheres ("sports ball") but a flat-yellow box doesn't read as a banana — proven empirically. Will need either YOLO-World (open-vocab) or photorealistic Fuel models for richer demos.
- Camera at 6.6 cm sees mostly floor + bottom of objects; works for floor objects (per `MISSION.md` scope) but not for things on counters/shelves.

Next up in Phase 2:
- [ ] More realistic home environment (AWS RoboMaker small house + Fuel models)
- [ ] Open-vocab detection (YOLO-World, then GroundingDINO when on GPU)
- [ ] Build a persistent semantic layer ("the red ball is at (-2.5, -1.0)") so it doesn't have to re-scan every time

---

## Phase 1 — Navigation — DONE (commit `91ed5c0`, 2026-05-02)

End-to-end autonomous SLAM + navigation working in `small_house.sdf`.

- **Sim core** — Gazebo Harmonic + ros_gz bridge + `VelocityControl` plugin (holonomic) + `OdometryPublisher`. URDF: `lekiwi.sim.urdf.xacro` includes the description's base + lidar but skips the pantilt module (its mount sits 5 cm above the LD06 plane and would otherwise show as a permanent obstacle).
- **Active SLAM** — `slam_toolbox` (async, omni-tuned) + Nav2 (no localization, slam supplies map) + `explore_lite` from `m-explore-ros2`. Frontier exploration drives a coverage tour, then `nav2_map_server map_saver_cli` dumps `small_house.{pgm,yaml}`.
- **Saved-map navigation** — Nav2 + AMCL with `set_initial_pose: true` to auto-localize at the spawn pose. MPPI controller in `Omni` mode for holonomic motion. Click "Nav2 Goal" in RViz → robot drives there.
- **Tuned configs** — `nav2_params.yaml` has all the omni-drive fixes: `motion_model: Omni`, `min_y_velocity_threshold: 0.001` (was 0.5 — diff-drive default), `velocity_smoother` Y components ±0.4 (was 0), `inflation_radius: 0.25` (sized for the 1.6 m hallway), `PreferForwardCritic` disabled.

Known issues from this phase:
- `active_slam.launch.py` "all-in-one" had unfixable DDS startup races on slow VMs and was removed. The multi-terminal recipe in [USAGE.md](USAGE.md) is the supported path.
- `slam_toolbox` can drop the first scan if it activates before `/scan` is flowing — staggered terminal startup (sim → wait → slam) sidesteps this.

---

## Pending phases

Per [MISSION.md](../MISSION.md):

- **Phase 3** — Real-robot validation. Workspace already includes `lekiwi_bringup/launch/lekiwi.launch.py` (Aditya's) for hardware bringup; Phase 1 + Phase 2 nodes should transfer with `use_sim_time:=false`. Needs a Pi 5 + LD06 + STS3215 motors + USB camera; clone `sts_hardware_interface`, `ldlidar_ros2`, and the camera driver into `src/`.
- **Phase 4** — SO-101 arm + teleop manipulation (LeRobot integration).
- **Phase 5** — LLM natural-language interface in front of `/target_color`.
- **Phase 6** — Sim-to-real: GPU box (or Jetson on the robot) for any Phase 2 perception that's CPU-bound today.

---

## Compute target

Today: developing in a UTM Linux VM on Apple Silicon Mac. CPU-only — UTM doesn't expose Metal as CUDA, and PyTorch's MPS backend isn't available in a Linux guest. YOLO at 2-4 Hz and Gazebo sensor rendering all run on CPU.

Migration path when CPU becomes the bottleneck (likely with GroundingDINO or richer worlds):
1. Bump UTM CPU allocation to 8+ cores (free, immediate)
2. Stand up a Linux + NVIDIA box (desktop ~$1000-1500) for development
3. Eventually a Jetson Orin on the robot itself for on-board perception

See README's "Tuning notes" for the full discussion.
