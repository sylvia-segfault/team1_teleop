open 3 terminals

Terminal 1:
```
cd ~/catkin_ws/
catkin build
source devel/setup.bash
stretch_robot_home.py
roslaunch team1_teleop websocket.launch
```

Terminal 2: \
`rosrun team1_teleop teleop_node`

Terminal 3: \
`python3 -m http.server`

Design: \
teleop_node subscribes to necessary info for finding and navigating to the walker.
teleop_node acts as a client that requests from the `find_nav_walker` service.
