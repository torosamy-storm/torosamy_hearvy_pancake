# bin_directory=/home/torosamy/develop-work-space/project/torosamy_little_pancake
bin_directory=$(pwd)
source install/setup.bash

cd $bin_directory/src/torosamy_automatic_aiming



if [ ! -d "build" ]; then
    mkdir build
fi
cd build
cmake ..
make
