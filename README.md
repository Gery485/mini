# mini

To understand how the complete project works, you should check out the following repositories:

- https://github.com/IntelRealSense/realsense-ros/wiki/SLAM-with-D435i
- https://git.uibk.ac.at/informatik/stair/minibot

If you have already read both repositories, you will know how to begin, but I will say every step again, as I also did.

1. Step Connection
Open a terminal and connect your device to the robot. Usually, you can use this command: ssh botuser@192.168.1.12 for the connection with the robot, but if you are using another router, you should check your IP address with the command ip -a.
After it you can start the docker container caller cpu with sudo docker start -i cpu.

2. Step set up everything
As first you will use tmux, so you can open 2 terminals in the same container easily. After this you should start the merged launch file with roslaunch (your directory) minibot_slam.launch. If started it you can go to the next step.

3. Step visualization
Congratulation, you are done with the hard part. As next you can open a terminal on your device for the vizualization. As first define here the rosmaster, that should be the next command: export ROS_MASTER_URI=http://192.168.1.12:11311, maybe you have to change the IP adress, if you are using an another router. Then navigate to launch folder within the my_robot_controller folder and roslaunch the namespaced_rviz.launch file. After this you only have to change the fixed Frame topic to map, but any other configuration shouldn´t be necessary.

4. Step navigation
Go back in the docker container ant start in the another terminal the moveTheRobotWithNodes.py. Normally you can use for this following command: rosrun my_robot_controller moveTheRobotsWithNodes.py

If you have done everything correctly it should look like something like this
![image](https://github.com/Gery485/mini/assets/139466090/831fa249-bf32-4ee9-acb6-fb684b84ff88)

To save your map you have several options but for the saving of a 2D map is the easiest way to use this: rosrun map_server map_saver map:=/minibot/rtabmap/proj_map -f name_of_your_map
Don´t forget to use this command in the folder, in that you also want to save the picture later on.

Besdie this, if you want to check the RAM of your robot, you have to connect the device with an another terminal to the robot and you can use the command htop. That will show you several data about the robot (microcontroller) and also the used RAM.

If you are done you can use the command exit to leave the docker container and you should you the command sudo poweroff, if you want to turn off the robot.
