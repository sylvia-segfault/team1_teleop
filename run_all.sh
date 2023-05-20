# !/bin/bash
cd ~/catkin_ws/
source devel/setup.bash
roslaunch team1_teleop f.launch & \
rosrun team1_teleop teleop_node & \
python3 -m http.server
