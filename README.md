# Safe Lane Selection

Students: Samuele Moranzoni, Ferdinando Giordano.

This project implements a camera-based lane-selection pipeline for a RoboMaster EP in CoppeliaSim and ROS 2. The road scene is intentionally clean and geometric, with a 200 m black road, two lanes, three continuous white lane boundaries, colored obstacles, and one RoboMaster EP named `rm0`.

## Aim

The project focuses on four main steps:

1. Ego-lane identification: given a multi-lane road, determine which lane the ego vehicle is currently occupying.
2. Orientation understanding: estimate whether the robot is aligned with the correct road direction.
3. Obstacle detection: detect colored obstacles through the camera sensor.
4. Path testing: choose and test a safe path on the road using the previous perception outputs.

The setup is a camera perception problem: detect lane lines, estimate lane centers, detect obstacles, visualize the robot's decisions, and control the robot with a simple feedback controller.

## ROS 2 Lane And Obstacle Architecture

The implementation is split into three ROS 2 nodes:

- `lane_perception_node`: subscribes to `/rm0/camera/image_color`, detects the lane geometry, and publishes `/lane_geometry`.
- `obstacle_perception_node`: subscribes to `/rm0/camera/image_color` and `/lane_geometry`, detects red/green obstacles, assigns them to a lane, and publishes `/obstacle_detection`.
- `lane_controller_node`: subscribes to `/lane_geometry` and `/obstacle_detection`, then publishes velocity commands on `/rm0/cmd_vel`.

Lane detection and obstacle detection run from camera frames, while the controller runs at a fixed control rate using the latest valid perception messages.

```text
/rm0/camera/image_color
   |
   +-- lane_perception_node --------> /lane_geometry
   |                                      |
   +-- obstacle_perception_node <---------+
   |             |
   |             +--------------------> /obstacle_detection
   |
   +-- lane_controller_node ---------> /rm0/cmd_vel
```

The geometry topic uses `std_msgs/Float32MultiArray` to keep the first implementation simple:

```text
/lane_geometry data layout:
[0] valid flag: 1.0 valid, 0.0 invalid
[1] vanishing point x in pixels
[2] vanishing point y in pixels
[3] heading error normalized by image width
[4] lateral error normalized by image width
[5] confidence in [0, 1]
[6] selected left lane boundary x at lookahead row
[7] selected right lane boundary x at lookahead row
[8] selected lane index: 0 left/current, 1 right, -1 unknown
[9] number of ordered road boundaries used, up to 3
[10] normalized vanishing-point spread from pairwise intersections
[11] lane width balance when 3 boundaries are visible
[12] x1, first boundary x at lookahead row
[13] x2, second boundary x at lookahead row
[14] x3, third boundary x at lookahead row, or -1 if missing
[15] left lane center x at lookahead row
[16] right lane center x at lookahead row, or -1 if missing
[17] selected lane center x at lookahead row
[18] current lane index: 0 left, 1 right, -1 unknown
[19] image width in pixels
[20] image center x in pixels
[21] first boundary slope m in x = m*y + b
[22] first boundary intercept b in x = m*y + b
[23] second boundary slope m
[24] second boundary intercept b
[25] third boundary slope m
[26] third boundary intercept b
```

The heading angle shown in the debug overlay is derived from the vanishing point
and the horizontal camera field of view:

```text
f_x = width / (2 * tan(FOV_x / 2))
theta = atan((VP_x - c_x) / f_x)
```

where `VP_x` is the vanishing-point x coordinate, `c_x` is the image center,
and `FOV_x` is set by the `camera_horizontal_fov_deg` parameter. The current
default is `90.0 deg`, so with a 640 px image `f_x = 320 px`.

The obstacle topic also uses `std_msgs/Float32MultiArray`:

```text
/obstacle_detection data layout:
[0] valid flag: 1.0 obstacle detected, 0.0 no obstacle
[1] color code: 1 red, 2 green, 0 none
[2] obstacle lane index: 0 left, 1 right, -1 unknown
[3] close flag: 1.0 close enough to avoid, 0.0 not close
[4] normalized obstacle area
[5] obstacle center x in pixels
[6] obstacle bottom y in pixels
[7] bounding-box x
[8] bounding-box y
[9] bounding-box width
[10] bounding-box height
```

The lane perception node also publishes an annotated debug camera stream:

```text
/lane_debug/image
```

Open it with:

```bash
ros2 run rqt_image_view rqt_image_view /lane_debug/image
```

The lane debug image shows:

- gray lines: all Hough candidate lane lines;
- colored lines: up to three ordered lane boundaries `x1`, `x2`, `x3`;
- yellow/green thick pair: the currently selected lane boundaries;
- blue vertical line: image center;
- blue horizontal line: lookahead row used for lane-center computation;
- red dot: vanishing point;
- `lane_detected`: current lane inferred from the image center and the lane boundaries;
- `vp`: vanishing point in pixels;
- `heading_error_norm`: horizontal vanishing-point error divided by image width;
- `angle`: approximate camera-space heading angle `theta` in degrees;
- `lateral_error_norm`: lane-center error divided by image width;
- `hough` and `boundaries`: detected Hough lines and selected road boundaries;
- `x=[x1, x2, x3]`: boundary intersections with the lookahead row;
- `lane_centers=[left, right]`: lane centers at the lookahead row.

The obstacle perception node publishes a second annotated debug stream:

```text
/obstacle_debug/image
```

Open it with:

```bash
ros2 run rqt_image_view rqt_image_view /obstacle_debug/image
```

The obstacle debug image shows:

- bounding box around the largest detected red or green obstacle;
- `obstacle`: detected color (`red`, `green`, or `none`);
- `lane`: lane assigned using the lane boundary equations at the obstacle bottom point;
- `close`: whether the obstacle is close enough to trigger avoidance;
- `area`: normalized detected blob area;
- `center=(cx, bottom_y)`: point used to assign the obstacle to a lane;
- `bbox=(x, y, w, h)`: raw image bounding box.

Useful Hough tuning parameters in `lane_following.launch.py`:

```text
hough_threshold: lower values accept weaker lines
hough_min_line_length_px: lower values accept shorter line fragments
hough_max_line_gap_px: higher values merge more fragmented line pieces
```

The current defaults are permissive:

```text
hough_threshold = 24
hough_min_line_length_px = 24.0
hough_max_line_gap_px = 35.0
```

## Build The Lane-Following Package

From this project folder:

```bash
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster
pixi shell
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/project_new
source /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster/install/setup.zsh
colcon build --symlink-install
source install/setup.zsh
```

## Run Camera Driver, Perception, And Control

Terminal 1: start CoppeliaSim, open the scene, and press Play.

```bash
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster
pixi shell
source install/setup.zsh
pixi run coppelia
```

Terminal 2: start the RoboMaster ROS driver with the raw camera topic enabled.

```bash
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster
pixi shell
source install/setup.zsh
ros2 launch robomaster_ros main.launch model:=ep name:=/rm0 \
  camera:=true video:=1 video_raw:=1 video_h264:=-1 video_ffmpeg:=-1 \
  video_resolution:=360 video_rate:=5.0 audio:=-1 \
  chassis:=true chassis_rate:=5 chassis_status_rate:=1 \
  arm:=false gripper:=false gimbal:=false led:=false speaker:=false \
  battery:=false armor:=false blaster:=false sbus:=false servo:=false \
  sensor_adapter:=false
```

Before launching the lane nodes, verify that the camera really has a publisher:

```bash
ros2 topic info -v /rm0/camera/image_color
```

The important line is:

```text
Publisher count: 1
```

If it says `Publisher count: 0`, the driver is not streaming camera frames yet. Restart the driver and make sure CoppeliaSim is in Play.

Terminal 3: build and run lane perception, obstacle perception, and control.

```bash
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster
pixi shell
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/project_new
source /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster/install/setup.zsh
colcon build --symlink-install
source install/setup.zsh
ros2 launch minimal_lane_following lane_following.launch.py
```

Terminal 4: watch the debug camera.

```bash
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster
pixi shell
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/project_new
source /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster/install/setup.zsh
source install/setup.zsh
ros2 run rqt_image_view rqt_image_view /lane_debug/image
```

Terminal 5: watch the obstacle debug camera.

```bash
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster
pixi shell
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/project_new
source /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster/install/setup.zsh
source install/setup.zsh
ros2 run rqt_image_view rqt_image_view /obstacle_debug/image
```

Useful checks:

```bash
ros2 node list
ros2 topic list | grep -E 'camera|lane|obstacle|cmd_vel'
ros2 topic info -v /rm0/camera/image_color
ros2 topic echo /lane_geometry
ros2 topic echo /obstacle_detection
ros2 topic hz /rm0/camera/image_color
```

Expected nodes:

```text
/lane_perception_node
/obstacle_perception_node
/lane_controller_node
```

Expected project topics:

```text
/lane_geometry
/lane_debug/image
/obstacle_detection
/obstacle_debug/image
/rm0/cmd_vel
```

Runtime parameters that are useful during tests:

```bash
# Force the controller target lane for debugging.
ros2 param set /lane_controller_node manual_target_lane left
ros2 param set /lane_controller_node manual_target_lane right
ros2 param set /lane_controller_node manual_target_lane current

# Disable or re-enable obstacle avoidance while keeping lane following active.
ros2 param set /lane_controller_node enable_obstacle_avoidance false
ros2 param set /lane_controller_node enable_obstacle_avoidance true

# Tune obstacle detection sensitivity.
ros2 param set /obstacle_perception_node min_area_ratio 0.003
ros2 param set /obstacle_perception_node close_area_ratio 0.025
ros2 param set /obstacle_perception_node close_bottom_ratio 0.72

# Keep this consistent with the CoppeliaSim vision sensor perspective angle.
ros2 param set /lane_perception_node camera_horizontal_fov_deg 90.0
```

If debug windows show no image, check the camera publisher first:

```bash
ros2 topic info -v /rm0/camera/image_color
```

`Publisher count` must be `1`. If it is `0`, the lane and obstacle nodes are alive but receive no camera frames.

## Scene To Use

Use this scene for the current lane-and-obstacle setup:

```text
/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project_new/scenes/scene_19/final_with_obstacles.ttt
```

In CoppeliaSim:

```text
File > Open scene...
```

Then open `final_with_obstacles.ttt`, press Play, and launch the ROS driver and nodes from the commands above.

Only if the scene needs to be regenerated or tested from scratch, run the fallback scene-generation script below.

## Generate A Fallback Scene ( only if you should have problem with final_with_obstacles.ttt )

Open CoppeliaSim from the RoboMaster lab environment:

```bash
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster
pixi shell
source install/setup.zsh
pixi run coppelia
```

Then, inside CoppeliaSim, open:

```text
Modules > Developer tools > Commander > Commander
```

Paste and run:

```lua
dofile('/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project_new/scripts/create_minimal_two_lane_scene.lua')
```

The fallback script saves the generated scene here:

```text
/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project_new/scenes/minimal_two_lane_scene.ttt
```

## Open The Generated Fallback Scene Later

In CoppeliaSim:

```text
File > Open scene...
```

Open the generated fallback scene:

```text
/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project_new/scenes/minimal_two_lane_scene.ttt
```

## What To Check Visually

In the active scene, the hierarchy should contain the RoboMaster and the road/camera elements used by the ROS driver:

- `/rm0`
- `/high_lane_camera`
- road surface and lane-boundary objects
- red/green obstacle objects

The elevated camera is placed behind and above the robot, tilted downward toward the lane markings. If the camera frame needs a small adjustment, select `/high_lane_camera` and rotate it slightly until the two-lane road is centered in view.
