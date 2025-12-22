#!/bin/bash

ros2 run controller_manager spawner -p scenario_controllers.yaml -c /b_controlled_box_cm joint_state_broadcaster --inactive

ros2 control set_hardware_component_state -c /b_controlled_box_cm kassow inactive
ros2 control switch_controllers -c /b_controlled_box_cm --activate joint_state_broadcaster

ros2 control set_hardware_component_state -c /b_controlled_box_cm kassow active


ros2 run controller_manager spawner -p scenario_controllers.yaml -c /b_controlled_box_cm kassow_joint_trajectory_controller --inactive
ros2 control switch_controllers -c /b_controlled_box_cm --activate kassow_joint_trajectory_controller

