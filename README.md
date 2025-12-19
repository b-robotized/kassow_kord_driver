# kassow kord driver

This repository contains the configuration and setup for Kassow Kord control, which demonstrates the use of a b»controlled box for robotic tasks.

> [!WARNING]
> This repository is under heavy development. If some links break or if some instruction do not lead to a smooth experience, please contact the developers.

Quick links:
- Setup container for packages: [Setup container](#setup-container)
- Quick demo to test the installation works: [Quick test of the setup](#quick-test-of-the-setup)

## Overview

This package provides a complete software solution for controlling Kassow Kord Robot (KR810) with full hardware integration and motion planning capabilities. The package implements a custom hardware interface compatible with ros2_control, enabling seamless operation of both physical robot and simulated one.

** Primary Capabilities:**

- Control of Kassow KR810 collaborative robots through a custom hardware interface and Joint Trajectory Controller
- Multi-robot setup to control both real and simulated robot simultaneously or independently on b»controlled box.
- Integration with MoveIt 2 for motion planning and trajectory execution

## Workspace setup

### 1. Clone all repositories from the `.repos` file

Make sure you have `python3-vcstool` installed:

```bash
sudo apt update
```

From the root of your workspace (e.g., `/kassow_kord_ws` -> easily direct to ws folder with the command `rosd`), run:

```bash
vcs import src < kassow_kord_driver.jazzy.repos
```

This will clone all repositories listed in the `kassow_kord_driver.jazzy.repos` file into the `src` directory.

### 2. Install ROS 2 dependencies with `rosdepi`

Update rosdep

```bash
rosdep update
```

Install dependencies

```bash
rosdepi
```

### 3. Build the workspace

Use `rtw` to build the workspace:

```bash
cb
```

After building, source the workspace:

```bash
source install/setup.bash
```

Now your development container should be ready for use.

## Running robot or simulation independently on ctrlX

1. Echo in a terminal the output of the `activity` topic from b»controlled box to observe its internal state.
   ```
   ros2 topic echo /b_controlled_box_cm/activity
   ```

2. To start the robot driver on the b»controlled box, choose one of the following:

   a. Run on kassow simulation
      ```
      ros2 launch kassow_kord_bringup kassow_kord_description.launch.xml use_mock_hardware:=false
      ```

   b. Run on kassow robot
      ```
      ros2 launch kassow_kord_bringup kassow_kord_description.launch.xml use_mock_hardware:=false ip_address:=10.23.23.204
      ```
      *Note: ensure MotionApp is in state `RUNNING` before activating hardware.*

    *Now you should see output on the `activity` with `unconfigured` hardware interfaces.*

3. In a new terminal, load the controllers, activate the hardware and enable control. Execute the commands form the `scripts` folder.
   ```
   rosd kassow_kord_bringup && cd scripts  # enter the correct folder
   ./activate_kassow_robot.bash  # follow the output on the activity topic
   ```

4. Start path planner (MoveIt2) and visualization (RViz 2):
   ```
   ros2 launch kassow_kord_bringup kassow_kord_moveit.launch.xml
   ```
   *MoveIt and visualisation can be started as soon as the hardware is active and Joint State Broadcaster is activeated.*

5. Now you can move the robot around either with moveit rviz plugin with motion planning, or you can directly use JTC CLI or rqt:
  ```
  ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
  ```

6. To stop the drivers of the robot that are running on b»controlled box use the following command:
   ```
   ./dectivate_pssbl_robot.bash  # follow the output on the activity topic
   ```

## Running both robot and simulation simultaneously

### Setup

single arm: kassow
multi arm: kassow_left, kassow_right.

### MockHw





### pc + kassow sim/robot

1. sim: `ros2 launch kassow_kord_bringup kassow_kord_bringup.launch.xml use_mock_hardware:=false`
   robot: `ros2 launch kassow_kord_bringup kassow_kord_bringup.launch.xml use_mock_hardware:=false ip_address:=10.23.23.204`

## Resources

- https://www.kassowrobots.com/downloads/product-manuals
- https://kassowrobots.gitlab.io/kord-api-doc/index.html
- https://gitlab.com/kassowrobots/kord-api

# Notes:

- install rtw
- load app
- sim ip: 10.28.28.11
- robot ip: 10.23.23.204
- port thing
- add scenario controllers to config
