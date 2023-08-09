# Running the UR5 Robot to Perform Grasping Applications
1. Start the UR5 robot based on the previous instructions.Make sure that the Robot is sending and recieving messeges using ```ping```. 
2. Make sure that the realsense camera is plugged in. This can be tested using the following command.
```
roslaunch realsense2_camera rs_camera.launch
```
Make sure all the python files in the scripts folder are executable. 
```vision_realsense.py``` is used to detect objects in a scene. There are two parameters that it must be given. ```-m``` which takes in two values namely ```real``` which is used to control real robot and the camera and ```sim``` which is used for the simulated robot.```-t``` takes in the target object that we want to detect. In our example we took "bottle" as our object. Run the following command after placing the realsense camera on the ```wrist_3_link``` of the ur5 Robot.
```
rosrun image2position vision_realsense.py -m real -t bottle
``` 
This command will check if the "bottle" object is detected in a particular frame.
It would give the following information about the bottle

The following topics will be launched
3. To visualize the robot model in rviz, run the following command
```
roslaunch ur5e_moveit_config moveit_rviz.launch config:=true
```
The simulated Robot will be displayed. 
4. To perform the segmentation on the data published in the ```sensor_msgs/Image``` topic and find the grasping point, run the following command
```
rosrun image2position vision_ros.py
```
The following nodes and topics would be launched. 
5. To implement grasping, run the following command
```
rosrun image2position ur5_commander.py -m real
```
After running the command, the following nodes and topics will be launched
In the terminal window you will be required to enter a few requests by the program. They include:
We can see the robot in the simulation perform the grasping operation on the robot on the calculated grasping points, and finally drop the object.
