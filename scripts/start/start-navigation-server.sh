bin_directory=/home/nuc/WorkSpace/project/torosamy_little_pancake

cd $bin_directory
source install/setup.bash

ros2 launch torosamy_navigation_server_launcher start.py \
map:=base \
mode:=nav \
robot:=torosamy_little_pancake \
localization:=slam \
rviz:=true \
