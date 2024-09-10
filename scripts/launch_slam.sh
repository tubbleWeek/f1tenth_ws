#!/bin/bash

source /opt/ros/foxy/setup.bash
source /$HOME/f1tenth_ws/install/setup.bash

ros2 launch slam_toolbox online_async_launch.py params_file:=/$HOME/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml

wait
