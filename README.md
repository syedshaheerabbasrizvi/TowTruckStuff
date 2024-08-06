first install ros 2 humble on ubuntu 22 and make a colcon workspace -- refer to the wiki for this

if bash complains about not recognizing ros2, try  source /opt/ros/humble/install/setup.bash 

cd ~/ros2_ws/src # this is where packages are built

ros2 pkg create --build-type ament_python --license Apache-2.0 py_pubsub # create a new package named py_pubsub with the license mentioned. feel free to change the name since it's not really a good one (you'll be seeing more poorly chosen names sadly since I didn't want to tinker around with default tutorial names until I got success)

cd ~/ros2_ws/src/py_pubsub/py_pubsub # go to the Python package of the ROS 2 package, both with the same names for some reason

gedit hussaincode.py # paste the content of the file with the same name in this repo; do the same for all gedit commands here. hussaincode.py connects the Arduino with ROS 2 and reads the frequency of the wheel encoder signal

gedit subscriber_member_function.py # this calculates and updates x, y, and theta using two inputs: the frequency of the signal (which is used to extract wheel speed) and the steering angle (currently hardcoded, will be implemented later). consider it as a kinematic calculator.

gedit pose_pub.py # this takes the position and orientation data, and publishes the pose in a format that RViz2 can directly use for display.

cd .. # go one directory backwards

gedit package.xml # we will be editing this xml file to add the dependencies we need during execution, and for boring description/maintainer/license info

gedit setup.py # we also edit this to add something known as an entry point, which is basically the names we give our files when we run them in ROS 2

cd ~/ ros2_ws # go to the root

rosdep install -i --from-path src --rosdistro humble -y # check if dependencies have been taken care of

colcon build --packages-select py_pubsub # build the entire thing! you need to do this every time you make a change in the files

after this point, please open Arduino IDE, connect with your Arduino, and upload the code given in wheel_encoder.ino (you might want to look for an undercase comment regarding hardcoded frequency). verify it's working with serial monitor, then close serial monitor.

source install/setup.bash # source the setup files (basically telling bash where your setup files are)

ros2 run py_pubsub hussain # run the hussain code, similar happens in the next several liens

(open new terminal, don't close old one!)

source install/setup.bash

ros2 run py_pubsub listener

(and again a new terminal)

source install/setup.bash

ros2 run py_pubsub poser

(and finally one more terminal)

source install/setup.bash

ros2 run rviz2 rviz2

now add a new pose and add the topic, voila

