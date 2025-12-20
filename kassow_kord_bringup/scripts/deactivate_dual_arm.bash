#!/bin/bash

ros2 control set_hardware_component_state -c /b_controlled_box_cm kassow_right inactive
ros2 control set_hardware_component_state -c /b_controlled_box_cm kassow_left inactive
