# bin_directory=/home/torosamy/develop-work-space/project/torosamy_little_pancake
bin_directory=$(pwd)

cd $bin_directory


colcon build --packages-skip torosamy_navigation_client torosamy_serial_port torosamy_automatic_aiming --symlink-install --executor sequential

# colcon build --packages-select livox_ros_driver2 --symlink-install --executor sequential