set -x
cd ..
dir
echo "Building the environment"
cd ros
catkin_make
dir
source devel/setup.sh
echo "Launching the nodes"
roslaunch launch/styx.launch