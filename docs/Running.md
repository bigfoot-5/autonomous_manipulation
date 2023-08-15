# Running the UR5 Robot to Perform Grasping Applications
1. Start the UR5 robot based on the instructions mentioned in [UR5Real.md](UR5Real).Make sure that the Robot is sending and recieving messeges using 
```
ping -<IP address of the robot>
```
2. Run the calibration script generated following the steps in [Calibration.md](Calibration.md).
3. Make sure that the realsense camera is plugged in.
Make sure all the python files in the scripts folder are executable. 
`vision_realsense.py` is used to detect objects in a scene. There are two parameters that it must be given. `-m` which takes in two values namely ```real``` which is used to control real robot and the camera and ```sim``` which is used for the simulated robot.```-t``` takes in the target object that we want to detect. In our example we took "bottle" as our object. Run the following command after placing the realsense camera on the ```wrist_3_link``` of the ur5 Robot.
```
rosrun image2position vision_realsense.py -m real -t bottle
``` 
This command will check if the "bottle" object is detected in a particular frame.

4. To visualize the robot model in rviz, run the following command
```
roslaunch ur5e_moveit_config moveit_rviz.launch
```
The simulated Robot will be displayed. 

5. To implement object detection and grasping, run the following command
```
rosrun image2position ur5_commander.py -m real
```
After running the command, there will be prompts to 
- Move the end effector to the grasping position
- grasp the object
- Move it to the second location
- drop the object
- Return to the home position
