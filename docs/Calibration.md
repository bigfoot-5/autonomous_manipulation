# Callibration testing
## Build Moveit Callibration Repository
Clone the Moveit_Callibration repository in the ```src``` folder in the workspace.
```
git clone git@github.com:ros-planning/moveit_calibration.git
```
### Install dependencies and build
```
rosdep install -y --from-paths . --ignore-src --rosdistro melodic
catkin_make
source devel/setup.sh
```
## Clone realsense plugin and set it on the UR5 arm
1. In the ```src``` folder clone the repository.
```
git clone git@github.com:IntelRealSense/realsense-ros.git
cd ..
catkin_make
source ./devel/setup.bash
```
2. Change the corresponding xacro file to insert the camera onto the robotic arm.
## Callibration using rviz
1. Launch the robot model on rviz
```
roslaunch ur5e_moveit_config demo.launch
```
2. Click the ```HandEyeCallibration``` panel in the display type.
3. Create the AruCo Board as a Target and Print it
4. Fill in the fields for the Geometric Context. 
5. Collect Data samples for Callibration
6. Calculate Callibration.
