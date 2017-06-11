# robotics_mini_project
Mini project in roboticas &amp; AI. Working with ROS, gazebo, openCV (python).

# Install
make sure you have ~/catkin_ws and all the usual ric reqs
clone https://github.com/robotican/ric into catkin_ws/src/ric
download this folder into catkin_ws/src/project
make sure you have in .bashrc : source ~/catkin_ws/devel/setup.bash
run: catkin_make 
run: rospack list, make sure you see "ric" & “project”

# Launch
cd ~/catkin_ws/src/project
launch.py (or: python launch.py)

Note: the launch files automatically runs the the command:
roslaunch project our_komodo.launch
with random position for the robot, in a random world(= random cube orders) and then run our_robot.py which handles the camera and laser messages.

