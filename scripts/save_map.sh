#!/bin/bash

source /opt/ros/foxy/setup.bash
source /$HOME/f1tenth_ws/install/setup.bash

ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: test, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"

wait
