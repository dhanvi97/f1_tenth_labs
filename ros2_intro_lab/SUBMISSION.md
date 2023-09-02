# Lab 1: Intro to ROS 2

## Written Questions

### Q1: During this assignment, you've probably ran these two following commands at some point: ```source /opt/ros/foxy/setup.bash``` and ```source install/local_setup.bash```. Functionally what is the difference between the two?

Answer: The ```opt/ros/foxy/setup.bash``` script allows terminal access to basic ros2 commands and packages installed with the foxy apt-get install. On the other hand, the ```install/setup.bash``` script in a local workspace is generated at build time to give terminal access to the newly created packages by the user.

### Q2: What does the ```queue_size``` argument control when creating a subscriber or a publisher? How does different ```queue_size``` affect how messages are handled?

Answer: Since node-to-node transportation of messages is an expensive process, ROS uses a queue system to allow for code execution asynchronously while the messages are sent. The ```queue_size``` parameter controls how many messages are stored for sending/receiving. The higher the queue_size, the lower the chances of missed messages due to process delays. But keeping it too high would result in larger memory consumption, especially for heavy messages like images or point cloud data.

### Q3: Do you have to call ```colcon build``` again after you've changed a launch file in your package? (Hint: consider two cases: calling ```ros2 launch``` in the directory where the launch file is, and calling it when the launch file is installed with the package.)

Answer: Since the launch file is a python script, if it is launched from the directory it does not need to be rebuilt since python packages are interpreted at run-time. However, if the launch file is installed via CMakeLists.txt the actual runnable launch file is in the install directory which means that the package will need to be rebuilt and sourced every time for the changes to be reflected