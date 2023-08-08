
1. Create a configuration with Robotiq gripper attached with UR5 robot:-

	* If not done already, clone the _universal_robots_ repository: Create a catkin workspace. Then, clone the repository within _src_ folder in the _ur5_ws_ workspace.
 	* Clone the robotiq repository.
		The robotiq repository hosted on the official account of ros-industrial Github organisation does not provide support for ROS noetic. However, a modified repository with required support is provided at https://github.com/jr-robotics/robotiq. 

```
git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git
```
```
git clone https://github.com/jr-robotics/robotiq.git
```
 

2. Combine the UR5 arm and the gripper using a URDF file.
3. Start a UR5 with gripper in Rviz
4. Control it with Rviz GUI as well as with terminal command line tool



References:
1. https://roboticscasual.com/ros-tutorial-how-to-create-a-moveit-config-for-the-ur5-and-a-gripper/
2. https://github.com/jr-robotics/robotiq
