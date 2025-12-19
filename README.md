# kassow kord driver

ros2 control hardware interface for kassow kord robot.

![Licence](https://img.shields.io/badge/License-Apache-2.0-blue.svg)

Pluginlib-Library: kassow_kord_hardware_interface
Plugin: kassow_kord_hardware_interface/KassowKordHardwareInterface (hardware_interface::SystemInterface)

## How to use

### mock hw

`ros2 launch kassow_kord_bringup kassow_kord_offline.launch.xml`

### pc + kassow sim/robot

1. sim: `ros2 launch kassow_kord_bringup kassow_kord_bringup.launch.xml hardware:=kr810`
   robot: `ros2 launch kassow_kord_bringup kassow_kord_bringup.launch.xml hardware:=kr810 ip_address:=10.23.23.204`

2. To move, you have to options:
    a. move with rqt: `ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller`
    b. move with cli: make sure to update joint limits to be close to current
    ```bash
    ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
    trajectory:
      joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7]
      points:
        - positions: [1.609778, 0.067324, 1.497803, 0.010716, -1.595989, 1.50, 2.09592]
          velocities: []
          time_from_start:
            sec: 3
            nanosec: 0
    "
    ```

### ctrlX + kassow sim/robot

1. sim: `ros2 launch kassow_kord_bringup kassow_kord_description.launch.xml hardware:=kr810`
   robot: `ros2 launch kassow_kord_bringup kassow_kord_description.launch.xml hardware:=kr810 ip_address:=10.23.23.204`
2. put on operating mode
3. activate hardware: `ros2 control set_hardware_component_state -c /b_controlled_box_cm kassow_kord_control active`
4. spawn joint state broadcaster: `ros2 run controller_manager spawner -c /b_controlled_box_cm -p ./src/kassow_kord_driver/kassow_kord_bringup/config/kassow_kord_controllers.yaml joint_state_broadcaster`
5. spawn joint trajectory controller: `ros2 run controller_manager spawner -c /b_controlled_box_cm -p ./src/kassow_kord_driver/kassow_kord_bringup/config/kassow_kord_controllers.yaml joint_trajectory_controller`
6. move: make sure to update joint limits to be close to current
    ```
    ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
    trajectory:
      joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7]
      points:
        - positions: [1.609778, 0.067324, 1.497803, 0.010716, -1.595989, 1.50, 2.09592]
          velocities: []
          time_from_start:
            sec: 3
            nanosec: 0
    "
    ```

## Resources

- https://www.kassowrobots.com/downloads/product-manuals
- https://kassowrobots.gitlab.io/kord-api-doc/index.html
- https://gitlab.com/kassowrobots/kord-api
