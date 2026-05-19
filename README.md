# Minimal Two-Lane Scene

This folder contains a clean CoppeliaSim setup for the rebuilt lane-following project.

## Goal

The scene is intentionally minimal:

- one black road;
- a longer 24 m road to make line convergence easier to observe;
- two lanes;
- three continuous white lane boundaries;
- one RoboMaster EP named `rm0`;
- one elevated downward-looking vision sensor named `high_lane_camera`.

This setup is meant to make the next step a simple camera-only geometric perception problem: detect the lane lines, estimate the lane center, and later control the robot with a PID controller.

## ROS 2 Lane-Following Architecture

The implementation is split into two independent ROS 2 nodes:

- `lane_perception_node`: subscribes to `/rm0/camera/image_color`, detects the lane geometry, and publishes `/lane_geometry`.
- `lane_controller_node`: subscribes to `/lane_geometry` and publishes velocity commands on `/rm0/cmd_vel`.

This keeps image processing separated from control. The perception node runs whenever camera frames arrive, while the controller runs at a fixed control rate using the latest valid geometry estimate.

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
```

The perception node also publishes an annotated debug camera stream:

```text
/lane_debug/image
```

Open it with:

```bash
ros2 run rqt_image_view rqt_image_view /lane_debug/image
```

The debug image shows the selected lane lines, the image center, the lookahead row, the vanishing point, and the numeric control values.

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

## Run Camera Driver And Lane Nodes

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
  camera:=true video:=1 video_raw:=1 video_h264:=0 video_ffmpeg:=0 \
  video_resolution:=360 video_rate:=10.0 audio:=-1 \
  chassis:=true chassis_rate:=10 chassis_status_rate:=1 \
  arm:=false gripper:=false gimbal:=false led:=false speaker:=false \
  battery:=true armor:=false blaster:=false sbus:=false servo:=false \
  sensor_adapter:=false
```

Terminal 3: run perception and control.

```bash
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster
pixi shell
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/project_new
source /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster/install/setup.zsh
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

Useful checks:

```bash
ros2 topic list | grep -E 'camera|lane|cmd_vel'
ros2 topic echo /lane_geometry
ros2 topic hz /rm0/camera/image_color
```

## Generate The Scene

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

The script saves the generated scene here:

```text
/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project_new/scenes/minimal_two_lane_scene.ttt
```

## Open The Scene Later

In CoppeliaSim:

```text
File > Open scene...
```

Open:

```text
/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project_new/scenes/minimal_two_lane_scene.ttt
```

## What To Check Visually

After generation, the scene hierarchy should contain:

- `/rm0`
- `/MinimalTwoLaneRoad`
- `/high_lane_camera`
- `/road_surface`
- `/white_lane_boundary_1`
- `/white_lane_boundary_2`
- `/white_lane_boundary_3`

The elevated camera is placed behind and above the robot, tilted downward toward the lane markings. If the camera frame needs a small adjustment, select `/high_lane_camera` and rotate it slightly until the two-lane road is centered in view.
