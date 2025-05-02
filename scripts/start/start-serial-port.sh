#!/bin/bash

# gnome-terminal -- bash -c "/home/ubuntu/workspace/project/torosamy_hearvy_pancake/start-serial-port.sh; bash"

bin_directory=/home/ubuntu/workspace/project/torosamy_hearvy_pancake

cd $bin_directory
source install/setup.bash

./src/torosamy_serial_port/build/torosamy_serial_port