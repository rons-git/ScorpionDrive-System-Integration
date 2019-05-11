set -ex
cd ..
cd ros
source devel/setup.bash
echo "Building the environment"
catkin_make
echo "Running roscore"
roscore &
sleep 5
echo "Launching the nodes"
roslaunch /capstone/ros/launch/styx.launch &
sleep 15
read -p "Press enter to exit"