set -x
cd ..
echo "Building the environment"
cd ros
catkin_make
source devel/setup.sh
echo "Launching the nodes"
roslaunch launch/styx.launch