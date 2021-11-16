How to start the robot using the ros to grab the lidar data and the smart matlab stuff:
1) run "roscore" in one teminal on the robot
2) in a new terminal ssh into the robot "ssh -X robotics@192.168.10.200" password robotics
2) cd mae_412_pkg
3) source devel/setup.bash
4) roslaunch mae_412_ws cam_aruco.launch (this will run the camera and aruco detection)
5) new terminal and ssh again
4) roslaunch smart2_bring_up smart2.launch
5) new terminal and ssh again
6) roslaunch smart2_bring_up joy_teleop.launch (this will let you use a joystick if one is plugged in)
