#!/bin/bash

source /opt/ros/foxy/setup.bash
source /$HOME/f1tenth_ws/install/setup.bash

ros2 run nav2_map_server map_server --ros-args --params-file /home/nvidia/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/map_server_params.yaml

ros2 run nav2_util lifecycle_bringup map_server
ros2 run nav2_amcl amcl --ros-args --params-file /home/nvidia/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/amcl_params.yaml

ros2 run nav2_util lifecycle_bringup amcl
