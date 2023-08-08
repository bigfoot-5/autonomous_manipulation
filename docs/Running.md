# Running the UR5 Robot to Perform Grasping Applications
1. Start the UR5 robot based on the previous instructions. Make sure that the Robot is sending and receiving data using ssh. 
2. Run the below command after connecting to the realsense camera.
```
roslaunch realsense2_camera rs_camera.launch
```
 The following topics will be launched
3. To visualize the robot model in rviz, run the following command
```
roslaunch ur5e_moveit_config demo.launch
```
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
