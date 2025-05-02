#!/bin/bash

# gnome-terminal -- bash -c "/home/ubuntu/workspace/project/torosamy_hearvy_pancake/start-navigation-client.sh; bash"

bin_directory=/home/ubuntu/workspace/project/torosamy_hearvy_pancake

cd $bin_directory
source install/setup.bash

./src/torosamy_navigation/torosamy_navigation_client/build/torosamy_navigation_client