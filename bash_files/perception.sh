#!/bin/bash
echo "perception.sh"

gnome-terminal -- sshpass -p 'locobot'  ssh -t locobot@10.15.3.56 "source /opt/ros/noetic/setup.sh; source /home/locobot/interbotix_ws/devel/setup.bash; python /home/locobot/interbotix_ws/src/interbotix_ros_rovers/interbotix_ros_xslocobots/interbotix_xslocobots_perception/scripts/perception.py; sleep 5"
