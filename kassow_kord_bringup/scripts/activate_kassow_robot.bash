#!/bin/bash

ros2 control set_hardware_component_state -c /b_controlled_box_cm kassow_kord_control active

ros2 run controller_manager spawner -c /b_controlled_box_cm -p ./src/kassow_kord_driver/kassow_kord_bringup/config/scenario_controllers.yaml joint_state_broadcaster
ros2 run controller_manager spawner -c /b_controlled_box_cm -p ./src/kassow_kord_driver/kassow_kord_bringup/config/scenario_controllers.yaml joint_trajectory_controller
