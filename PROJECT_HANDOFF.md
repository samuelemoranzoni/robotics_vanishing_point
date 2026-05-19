# Project Handoff - Minimal Camera-Based Lane Following

This file summarizes the new clean project so a new conversation can continue from the right context.

## Current Goal

We are rebuilding the previous Safe Lane Selection project from scratch, with a much simpler and more explainable setup.

The new project is a minimal camera-only lane perception problem:

- one RoboMaster EP in CoppeliaSim;
- one long black road;
- two lanes;
- three continuous white lane boundaries:
  - left road/lane boundary;
  - center divider;
  - right road/lane boundary;
- no obstacles for now;
- no safe lane selection for now;
- focus only on geometric lane perception and later PID lane centering.

The professor suggested a geometric approach:

- detect the lane lines from the camera image;
- compute the lane line equations with a Hough transform or similar line fitting method;
- estimate the vanishing point from the intersection of lane lines;
- compute the robot heading/orientation error as the horizontal offset between the image center and the vanishing point;
- compute the lateral error from the robot position inside the lane, ideally using the two lane boundaries around the robot;
- use a PID controller to keep the robot centered in its lane.

Important: this is meant to be solved first geometrically, before adding obstacle detection.

## Working Directories

Main new project folder:

```text
/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project_new
```

Scene generation script:

```text
/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project_new/scripts/create_minimal_two_lane_scene.lua
```

Generated CoppeliaSim scene:

```text
/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project_new/scenes/minimal_two_lane_scene.ttt
```

RoboMaster lab workspace:

```text
/Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster
```

## Current Scene State

The scene script currently creates:

- a long road, currently `ROAD_LENGTH = 140.0`;
- two lanes, each with `LANE_WIDTH = 1.35`;
- three continuous white lane boundaries at `y = -LANE_WIDTH`, `y = 0`, and `y = +LANE_WIDTH`;
- a RoboMaster model named `/rm0`;
- a small cyan visual marker above the robot so it is easier to see from the external CoppeliaSim view;
- a separate debug camera named `/high_lane_camera`.

Important correction for the next step:

The final perception should use the RoboMaster camera, not `/high_lane_camera`.

The camera used by the RoboMaster ROS stream is the vision sensor inside `/rm0`, named:

```text
Vision_sensor
```

That is the camera streamed by the RoboMaster simulator plugin to:

```text
/rm0/camera/image_color
```

So the next implementation step should tune/mount the RoboMaster's own `Vision_sensor`, not create a separate camera.

## Camera Plan

The camera should be a raised dashcam-style view:

- mounted on or inside the RoboMaster model;
- slightly above the robot;
- behind/above the front body enough to see the road;
- tilted forward and downward;
- not top-down.

Why not top-down?

If the camera looks too much from above, the lane lines remain almost parallel in the image and the vanishing point goes to infinity. For the professor's geometric method, we need perspective: the three lane boundaries should visually converge toward a vanishing point.

The target camera image should show:

- the left boundary;
- the center divider;
- the right boundary;
- the three lines extending forward and converging in the distance.

## Desired Perception Algorithm

The intended perception pipeline should be:

1. Subscribe to the RoboMaster camera image:

```text
/rm0/camera/image_color
```

2. Convert image to HSV or grayscale.

3. Segment white lane markings.

4. Use Canny edges and Hough lines, or directly fit line models to lane pixels.

5. Separate the detected lines into:

- left boundary;
- center divider;
- right boundary.

6. Compute line equations in image coordinates:

```text
x = m*y + b
```

This form is convenient because lane lines are mostly vertical in the image.

7. Compute a vanishing point.

General idea:

- take two non-parallel lane lines;
- compute their intersection;
- optionally average intersections from multiple line pairs;
- reject unstable intersections outside a reasonable image region.

8. Compute orientation error:

```text
orientation_error = vanishing_point_x - image_center_x
```

If the vanishing point is on the image center, the robot is aligned with the road direction.

If the vanishing point is shifted left or right, the robot is rotated relative to the road.

9. Compute lateral error:

At a fixed lookahead row near the bottom of the image:

```text
left_x = x coordinate of left lane boundary
right_x = x coordinate of right lane boundary
lane_center_x = (left_x + right_x) / 2
lateral_error = lane_center_x - image_center_x
```

This measures whether the robot is centered between the two lane boundaries.

10. PID controller:

Use lateral error and orientation error to control angular velocity:

```text
angular_z = Kp_lat * lateral_error + Kp_head * orientation_error
```

The robot should move slowly forward while steering to keep the lane centered.

## Why This New Setup Is Simpler

The old project became too complicated because it mixed:

- many lanes;
- object detection;
- safe lane selection;
- ToF emergency stop;
- world-pose lane control;
- camera lane detection;
- marker localization;
- multiple control states.

The new project intentionally removes all of that and focuses on one explainable behavior:

The robot follows the center of a lane using only camera geometry.

This is easier to present and debug.

## How To Generate The Scene

Open CoppeliaSim from the RoboMaster lab environment:

```bash
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster
pixi shell
source install/setup.zsh
pixi run coppelia
```

Inside CoppeliaSim:

```text
Modules > Developer tools > Commander > Commander
```

Run:

```lua
dofile('/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project_new/scripts/create_minimal_two_lane_scene.lua')
```

The scene is saved at:

```text
/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project_new/scenes/minimal_two_lane_scene.ttt
```

## How To Open The Scene Later

In CoppeliaSim:

```text
File > Open scene...
```

Open:

```text
/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project_new/scenes/minimal_two_lane_scene.ttt
```

Then press Play.

If the RoboMaster connection becomes unstable, enable real-time mode in CoppeliaSim:

```text
Simulation > Real-time mode
```

## How To Launch The RoboMaster ROS Driver

Use a second terminal:

```bash
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster
pixi shell
source install/setup.zsh
export ROS_LOCALHOST_ONLY=1
ros2 launch robomaster_ros ep.launch name:=rm0
```

If `name:=rm0` is not accepted by that launch file, try:

```bash
ros2 launch robomaster_ros ep.launch
```

## How To View The RoboMaster Camera

Use a third terminal:

```bash
cd /Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster
pixi shell
source install/setup.zsh
export ROS_LOCALHOST_ONLY=1
ros2 run rqt_image_view rqt_image_view
```

In `rqt_image_view`, select:

```text
/rm0/camera/image_color
```

This is the camera image that should be used for lane detection.

## Important Current Issue

The scene currently still creates `/high_lane_camera`, but the user wants to use the RoboMaster camera stream instead.

Next coding step:

- remove or ignore `/high_lane_camera`;
- find the RoboMaster internal `Vision_sensor` after loading `/rm0`;
- move/rotate that `Vision_sensor` to a raised dashcam pose;
- keep it inside `/rm0`, so the simulator plugin still streams it as `/rm0/camera/image_color`.

Useful implementation clue:

The RoboMaster simulator plugin looks for a vision sensor named:

```text
Vision_sensor
```

The relevant plugin code is in:

```text
/Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster/src/robomaster_sim/coppeliaSim_plugin/plugin.cpp
```

It streams that sensor through the RoboMaster ROS camera topic.

## Troubleshooting Commands

If ROS topics look stale:

```bash
ros2 daemon stop
```

If CoppeliaSim or the simulator plugin is stuck:

```bash
pkill -f coppeliaSim
```

If the simulator says a port/address is already in use, close old CoppeliaSim instances and retry:

```bash
pkill -f coppeliaSim
```

Then reopen CoppeliaSim and launch again.

## Next Concrete Tasks

1. Patch `create_minimal_two_lane_scene.lua` so it tunes the RoboMaster `Vision_sensor` instead of creating `/high_lane_camera`.

2. Regenerate the scene.

3. Open `/rm0/camera/image_color` in `rqt_image_view`.

4. Adjust the RoboMaster camera pose until the three lane boundaries clearly converge toward a vanishing point.

5. Implement the minimal lane perception node:

- white lane segmentation;
- Hough line detection;
- line grouping;
- vanishing point computation;
- lane center computation;
- debug image.

6. Add a simple PID controller to keep the robot centered.

## Presentation Message

The project is a minimal autonomous lane-following system in simulation. The robot uses only its onboard camera to perceive a two-lane road. The lane markings are detected geometrically from the image, then the lane center and the vanishing point are estimated. The lane center provides the lateral error, while the vanishing point gives an orientation error. These two quantities can be used by a PID controller to keep the RoboMaster aligned and centered in its lane.

