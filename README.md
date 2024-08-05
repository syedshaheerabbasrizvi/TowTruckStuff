first install ros 2 humble on ubuntu 22 and make a colcon workspace -- refer to the wiki for this

if bash complains about not recognizing ros2, try  source /opt/ros/humble/install/setup.bash 

cd ~/ros2_ws

ros2 pkg create --build-type ament_python --license Apache-2.0 py_pubsub

cd ~/ros2_ws/src/py_pubsub/py_pubsub

gedit hussaincode.py # paste the content of the file with the same name in this repo; do the same for all gedit commands here

gedit subscriber_member_function.py

gedit pose_pub.py

cd ..

gedit package.xml

gedit setup.py

cd ~/ ros2_ws

rosdep install -i --from-path src --rosdistro humble -y

colcon build --packages-select py_pubsub

after this point, please open Arduino IDE, connect with your Arduino, and upload the code given in wheel_encoder.ino (you might want to look for an undercase comment regarding hardcoded frequency). verify it's working with serial monitor, then close serial monitor.

source install/setup.bash

ros2 run py_pubsub hussain

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

