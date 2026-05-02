# Usage cheatsheet

Practical commands for running the LeKiwi sim. Each major recipe assumes the workspace is built and `source ~/lekiwi_ws/install/setup.bash` is in your `~/.bashrc` (one-time setup in [SETUP.md](SETUP.md)).

If a relaunch ever shows ghost robots, wrong worlds, or stale Nav2 nodes, nuke everything first:
```bash
pkill -9 -f "gz sim|ruby.*gz|robot_state_pub|parameter_bridge|slam_toolbox|rviz2|ros2 launch|explore|yolo|go_to_color_ball"
```

---

## Just look at the URDF

```bash
ros2 launch lekiwi_description display.launch.py
```
RViz opens with the robot model + joint sliders. No sim, no Nav2.

---

## Drive the robot manually

Once any sim is running:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Keys: `i/,` forward/back · `j/l` rotate · `u/o`, `m/.` strafe diagonals · `k` stop · `q/z` faster/slower.
Terminal must have keyboard focus — clicking on Gazebo/RViz won't reach it.

---

## Recipe A — autonomous mapping (Phase 1)

Builds a fresh `/map` of the world by exploring frontiers automatically. Run each in its own terminal.

```bash
# T1 — sim
ros2 launch lekiwi_gazebo sim_bringup.launch.py world:=small_house.sdf

# T2 — slam_toolbox (wait for it to log "Activating", then check):
ros2 launch lekiwi_gazebo slam.launch.py
# Verify:
ros2 lifecycle get /slam_toolbox      # → active [3]
ros2 topic hz /map                    # → ~0.2 Hz

# T3 — Nav2 (no map_server / amcl; SLAM supplies the map)
ros2 launch lekiwi_navigation nav2.launch.py \
  use_localization:=False use_composition:=False
# Verify:
ros2 lifecycle get /bt_navigator      # → active [3]

# T4 — frontier explorer (auto-drives)
ros2 run explore_lite explore --ros-args \
  --params-file ~/lekiwi_ws/install/lekiwi_gazebo/share/lekiwi_gazebo/config/explore_lite.yaml \
  -p use_sim_time:=true \
  -r /tf:=tf -r /tf_static:=tf_static
```

When `explore_lite` logs `No frontiers found, stopping`, save the map:
```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/lekiwi_ws/src/lekiwi_ros2/lekiwi_navigation/maps/small_house \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=15.0
```

A reference `small_house.{pgm,yaml}` is already committed in the repo.

---

## Recipe B — saved-map navigation (Phase 1)

Drive to clicked goals on a previously-built map.

```bash
# T1 — sim
ros2 launch lekiwi_gazebo sim_bringup.launch.py world:=small_house.sdf

# T2 — Nav2 with AMCL (auto-localizes to the spawn pose -1.5, 0)
ros2 launch lekiwi_navigation nav2.launch.py \
  use_composition:=False
# (defaults to map=small_house.yaml)

# T3 — RViz with Nav2's prebuilt config (has the toolbar buttons)
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz \
  --ros-args -p use_sim_time:=true
```

In the RViz toolbar, click **Nav2 Goal** → click on the map → drag to set yaw → release.

Or send goals from the CLI:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}'
```

---

## Recipe C — "go to the X ball" (Phase 2)

Robot autonomously searches for a colored ball and stops in front of it. Builds on Recipe B.

```bash
# T1 — sim (small_house has yellow, red, blue balls on the living-room floor)
ros2 launch lekiwi_gazebo sim_bringup.launch.py world:=small_house.sdf

# T2 — Nav2 with AMCL
ros2 launch lekiwi_navigation nav2.launch.py use_composition:=False

# T3 — YOLOv8 detector on /camera/image
ros2 launch lekiwi_gazebo yolo.launch.py

# T4 — color-aware goal sender (the brain)
ros2 run lekiwi_gazebo go_to_color_ball.py
```

Trigger with:
```bash
ros2 topic pub --once /target_color std_msgs/String "{data: red}"
# also: yellow, blue
```

Watch the camera + detection feed:
```bash
ros2 run rqt_image_view rqt_image_view /yolo/dbg_image
```

The node will spin to scan, drive room-to-room if needed, navigate to the matching ball, and fine-adjust its orientation so the ball ends up centered in the camera view. Logs each step.

---

## Inspect / debug

```bash
# Topic discovery (refresh first if anything seems missing)
ros2 daemon stop && ros2 topic list

# Nav2 lifecycle status
for n in bt_navigator controller_server planner_server behavior_server; do
  printf "%-25s " "$n:"; ros2 lifecycle get /$n 2>&1 | tail -1
done

# Live data rates
ros2 topic hz /scan /odom /camera/image /yolo/detections /map

# Camera + detections live view
ros2 run rqt_image_view rqt_image_view /camera/image
ros2 run rqt_image_view rqt_image_view /yolo/dbg_image

# TF tree
ros2 run tf2_tools view_frames    # writes frames.pdf
```

---

## Worlds

| `world:=` | What's in it |
|---|---|
| `small_house.sdf` | 3-room apartment + sofa/bed/counter + 3 colored balls (default Phase 2 demo) |
| `empty.sdf` | Open floor with a few boxes for quick sanity tests |
| `depot.sdf` | Fuel-backed warehouse model — needs internet on first run |

Override spawn coords with `x:= y:=` for non-default worlds.
