# StickCatcherRobot_ME133

ME/CS/EE 133a final project - Atlas catches a randomly dropped stick.

Team: Priyanka, Jenny, Katie

## Milestone 1 (Tuesday) - no stepping

Atlas stands still. A stick is dropped from rest at a random (x, y)
inside the right arm's reach. The arm splines to the catch point and
waits; when the stick arrives, it snaps to the hand frame.

## Setup

Clone / place this package at `~/robotws/src/stickcatcher`, then copy
one file over from HW6:

    cp ~/robotws/src/hw6code/hw6code/KinematicChain.py \
       ~/robotws/src/stickcatcher/stickcatcher/

`TransformHelpers.py` and `TrajectoryUtils.py` are imported from the
`utils` package, so nothing to copy for those.

You do NOT need to copy the Atlas URDF or the RVIZ config for
Milestone 1 - just point the existing ones at this package. Find them
with:

    ls ~/robotws/src
    grep -rl "atlas" ~/robotws/src --include=*.urdf

Milestone 2 is when we need our own editable copy of the URDF, so the
virtual base can be added (see `urdf/virtualbase_snippet.urdf`).

Then build:

    cd ~/robotws
    colcon build --symlink-install
    source ~/robotws/install/setup.bash

## Run (four terminal tabs, as in HW3)

    cd ~/robotws/src/utils/rviz
    ros2 run rviz2 rviz2 -d viewatlas.rviz

    cd ~/robotws/src/<whichever package has it>/urdf
    ros2 run robot_state_publisher robot_state_publisher atlas.urdf

    ros2 run stickcatcher stick
    ros2 run stickcatcher catch

In RVIZ set the Fixed Frame to `world` and add a `MarkerArray` display
on `/visualization_marker_array` to see the stick.

## Files

| file | what it does |
|---|---|
| `stickcatcher/stick.py`  | the world: random spawn, ballistic fall, kinematic grab, CYLINDER marker |
| `stickcatcher/catch.py`  | the robot: right-arm inverse kinematics, splined reach, world->pelvis TF |
| `stickcatcher/KinematicChain.py` | copied from hw6code, unmodified |
| `urdf/virtualbase_snippet.urdf` | the 6-DOF floating base to paste into Atlas for Milestone 2 |

## Topics

    /stick/pose    PoseStamped    stick  -> catch
    /stick/twist   TwistStamped   stick  -> (unused until we predict)
    /hand          PoseStamped    catch  -> stick
    /joint_states  JointState     catch  -> robot_state_publisher

## TODO after Tuesday

- [ ] 6-DOF virtual base (world -> 3 prismatic -> 3 continuous -> pelvis)
- [ ] re-root the chains at the stance foot; stance-foot-fixed task
- [ ] swing-foot step spline + stance swap
- [ ] ballistic prediction so the stick can be thrown, not just dropped
- [ ] wrist orientation task (align the grasp axis with the stick axis)
- [ ] weighted pseudo-inverse / singularity handling near full extension
