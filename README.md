open 3 terminals

Terminal 1:
```
cd ~/catkin_ws/
catkin build
source devel/setup.bash
roslaunch team1_teleop websocket.launch
```

Terminal 2: \
`rosrun team1_teleop teleop_node`

Terminal 3: \
`python3 -m http.server`