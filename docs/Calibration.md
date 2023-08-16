# Callibration
Firstly connect the robot by following the steps mentioned in [UR5Real.md](UR5Real.md).
## Build Moveit Callibration Repository
Clone the Moveit_Callibration repository in the ```src``` folder in the workspace.
```
cd ur5_ws/src
git clone git@github.com:ros-planning/moveit_calibration.git
```
### Install dependencies and build
```
rosdep install -y --from-paths . --ignore-src --rosdistro melodic
cd ..
catkin_make
source devel/setup.sh
```
## Clone realsense plugin and set it on the UR5 arm
In the ```src``` folder clone the repository.
```
cd src
git clone git@github.com:IntelRealSense/realsense-ros.git
cd ..
catkin_make
source ./devel/setup.bash
```
## Callibration using rviz
1. Launch the robot model and camera on rviz after connecting the 
```
roslaunch ur5e_moveit_config moveit_rviz.launch
roslaunch realsense2_camera rs_camera.launch
```
2. Click the ```HandEyeCallibration``` panel in the display type after clicking "Add".
<img width="487" alt="image" src="https://github.com/bigfoot-5/autonomous_manipulation/assets/68162715/2048ef6d-fa11-4289-afc3-60478db948e6">

3. Click on the "Target" tab, and in the "Target Params" section enter the following details.
Select the "Image Topic" as `/camera/color/image_raw` and "Camera Info topic" as `/camera/color/camera_info` shown in the below image.

<img width="696" alt="image2" src="https://github.com/bigfoot-5/autonomous_manipulation/assets/68162715/7189625e-793c-4758-8b49-c2ed009e15b6">

Enter "Create Target", and then "Save Target". An Aruco board will be created, which could be printed on an A4 Sheet. Place it on the corner of the table connected to the base_link of the robot as shown in the below image 
![put an image here]().
4. Fill in the fields for the Geometric Context.
```change the below image later using the real one```
![geometric context]()
Place the printed AruCo board under the sensor of the UR5 arm. 
5. In the "Calibrate" tab, click load joint states and browse to the file named ```joint_states_camera_calibration.yaml``` which is in the path ```/automonous-mainpulation/config```. 
6. Now click `Plan`, and then click `Execute`. The first Sample will be taken. Repeat this step, until the Progress bar below reaches "100%". 
<img width="694" alt="image4" src="https://github.com/bigfoot-5/autonomous_manipulation/assets/68162715/654466ba-d7a2-464c-b678-d4d088134fea">

7. Finally click "Save camera pose" and save it in the launch folder as `camera_pose.launch`. This file will be used to callibrate the camera when planning grasping path towards the object.
