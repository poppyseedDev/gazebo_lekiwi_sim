# Mission

## Vision
Build a mobile manipulation robot — **LeKiwi** base with an **SO-101** arm on top — that can be told in natural language to fetch or move objects. Example end-to-end task: *"Go to the banana in the living room and move it to the pile."*

## End-to-End Behavior
1. User issues a natural-language command referencing an object and a location.
2. Robot localizes itself in a previously mapped environment.
3. Robot autonomously navigates to the target object using **LiDAR for localization/obstacle avoidance** and **camera for object detection**.
4. Once near the object, control hands off to **teleoperation** for manipulation (pick, place, sort).

## Phased Approach

### Phase 1 — Navigation (current)
- Simulate LeKiwi in **Gazebo** (Linux VM for now).
- Build a reliable **2D occupancy map** of a room using LiDAR-based SLAM.
- Achieve **autonomous point-to-point navigation** on the saved map (goal pose → robot drives there, avoids obstacles).
- Progressively add realism: clutter, dynamic obstacles, target objects (blocks, bananas, etc.).

### Phase 2 — Perception & Semantic Goals
- Run camera-based object detection (open-vocabulary).
- Fuse detections into a **semantic layer** on top of the occupancy map.
- Convert *"go to object X"* into a navigation goal pose.

### Phase 3 - Test this on a real robot
 - Remove the hand
 - test just camera + lidar
 - with object detection

### Phase 4 — Teleoperation & Manipulation
- Integrate SO-101 arm control via LeRobot.
- Teleoperate manipulation tasks once the robot has arrived at the object.

### Phase 5 — Language Interface
- LLM parses natural-language commands into structured goals (target object + action).

### Phase 6 — Sim-to-Real
- Transfer the working sim pipeline to physical LeKiwi + SO-101 hardware.

## Current Focus
- Get mapping working reliably in Gazebo (debugging SLAM Toolbox).
- Then bring up Nav2 for autonomous navigation on the saved map.
- Then start adding realistic objects and scenarios to the simulated world.

## Open Questions / Risks
- VM may become a bottleneck once camera + perception are added.
- Need to decide on object-detection stack for Phase 2.
- Need to plan early for sim-to-real gap (sensor noise, odometry drift, lighting).