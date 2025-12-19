#!/bin/bash

ros2 run controller_manager spawner -p scenario_controllers.yaml -c /b_controlled_box_cm joint_state_broadcaster

ros2 run controller_manager spawner -p scenario_controllers.yaml -c /b_controlled_box_cm kassow_left_joint_trajectory_controller --inactive
ros2 run controller_manager spawner -p scenario_controllers.yaml -c /b_controlled_box_cm kassow_right_joint_trajectory_controller --inactive

ros2 control switch_controller -c /b_controlled_box_cm --start-controllers kassow_left_joint_trajectory_controller
ros2 control switch_controller -c /b_controlled_box_cm --start-controllers kassow_right_joint_trajectory_controller