#!/bin/bash
echo "rosbag.sh"

gnome-terminal -- sshpass -p 'locobot'  ssh -t locobot@10.15.3.56 "source /opt/ros/noetic/setup.sh; source /home/locobot/interbotix_ws/devel/setup.bash; rosbag record -a -x "\(.*\)/camera/\(depth\|color/image_raw\|aligned_depth_to_color/image_raw\)/\(.*\)"; sleep 10"
