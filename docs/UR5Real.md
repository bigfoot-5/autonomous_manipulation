Setup the UR5 robot for ur_robot_driver:

1. Configure the hardware:
   
	i. Note the IP address of the robot[[2]](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial) from the UR teach-pendant by navigating to the Setup Robot -> Network. This shall be used to establish a connection with the pc in the following steps.

	ii. On the teach pendant - go to Program Robot -> Installation -> External Control: Confirm the Host IP; Type '50002' in the Custom port; type the Host name from the host pc account which shall be used to operate the robot.

	iii. Go to 'Program' tab; press 'Empty Program'; In the left section, below 'Robot Program', press <'empty'>.

	iv. In the 'Insert program lines here' section, press 'Structure'.

	v. Go to 'URCaps' tab and press on 'External Control'.

Now, the left section of the display, below 'Robot Program', will show 'Control by hostname', according to the hostname which is entered in previous point.

2. Extract the calibration information from the robot - this shall provide the current parameters of the robot in a 'yaml' file[[5]](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master#prepare-the-ros-pc):
```
roslaunch ur_calibration calibration_correction.launch robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"
```
3. Start the robot driver using the existing launch file and pass the calibration information along with it
```
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=<robot_ip> kinematics_config:="${HOME}/my_robot_calibration.yaml"
```
4. Use [MoveIt!](http://wiki.ros.org/action/show/moveit?action=show&redirect=MoveIt) to control the robot and allow motion planning[[2]](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial)
```
roslaunch ur5e_moveit_config moveit_planning_execution.launch
```
5. Start [RViz](http://wiki.ros.org/rviz), including the MoveIt! motion planning plugin run:
```
roslaunch ur5e_moveit_config moveit_planning_execution.launch
```
6. Use RViz interface to fix the goal state of robot by changing the required joint angles.
7. Click on plan - ensure the path is free of obstacles.
8. Click execute - verify the movement of UR5 joints.


Reference:

[1] Date of access - 07/07/2023
```
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
```
[2] Date of access - 07/07/2023
```
http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial
```
[3] Date of access - 07/07/2023
```
https://github.com/ros-industrial/universal_robot
```
[4] Date of access - 07/07/2023
```
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md#installing-a-urcap-on-a-cb3-robot
```
[5] Date of access - 07/07/2023
```
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master#prepare-the-ros-pc
```
