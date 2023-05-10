# !/bin/bash
cd ~/catkin_ws/
catkin build
source devel/setup.bash
# stretch_robot_home.py
roslaunch team1_teleop websocket.launch & \
rosrun team1_teleop teleop_node & \
python3 -m http.server
