#!/bin/bash
cd ..
pip install --upgrade pillow
apt-get install dos2unix
sleep 5
find . -name '*.py' -exec dos2unix {} \;
sleep 1
find . -name '*.sh' -exec dos2unix {} \;
sleep 1
cd ros/src
source /capstone/ros/devel/setup.bash
cd /capstone/ros
echo "Building the environment"
catkin_make
echo "Running roscore"
roscore &
sleep 5
echo "Launching the nodes"
roslaunch /capstone/ros/launch/styx.launch &
sleep 15
read -p "Press enter to stop"
cd ..
cd scripts
rm d2utmp* 2> /dev/null
read -p "Press enter to exit"