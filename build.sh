# bin_directory=/home/torosamy/develop-work-space/project/torosamy_little_pancake
bin_directory=$(pwd)
cd $bin_directory

rm -rf out

current_time=$(date +"%Y-%m-%d_%H-%M-%S")
echo "${current_time}" > version.txt
echo "${current_time}"

./scripts/build/build-navigation-server.sh

wait

./scripts/build/build-automatic_aiming.sh


wait

./scripts/build/build-navigation-client.sh



wait

./scripts/build/build-serial-port.sh
