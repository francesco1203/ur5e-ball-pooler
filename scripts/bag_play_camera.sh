#!/bin/bash

ROS_SETUP="/opt/ros/jazzy/setup.bash"
# BAG_DIR="data/bagdata/only_camera_logging_compressed/only_camera_logging_compressed_0.mcap"
BAG_DIR="data/bagdata/save_videos/due_palle_con_fallo/only_camera_logging_compressed_0.mcap"

# 1. Riproduzione Bag File
gnome-terminal --tab --title="Bag Playback" -- bash -c \
    "source ${ROS_SETUP} && \
     ros2 bag play ${BAG_DIR} --loop; \
     exec bash"

sleep 2

# 2. image_view con image_transport integrato
gnome-terminal --tab --title="Image View" -- bash -c \
    "source ${ROS_SETUP} && \
     ros2 run image_view image_view \
        --ros-args \
        -r image:=/camera/camera/color/image_raw \
        -p image_transport:=compressed; \
     exec bash"