#!/bin/bash

# gnome-terminal -- bash -c "/home/ubuntu/workspace/project/torosamy_hearvy_pancake/start-navigation-server.sh; bash"

bin_directory=/home/ubuntu/workspace/project/torosamy_hearvy_pancake

cd $bin_directory
source install/setup.bash

ros2 launch torosamy_navigation_server_launcher start.py \
map:=test \
robot:=torosamy_hearvy_pancake \
localization:=null \
rviz:=true \
