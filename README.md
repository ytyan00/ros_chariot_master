# ros_chariot_master


## 1. ros master & follower setup
##### master
conda deactivate
export ROS_MASTER_URI=http://192.168.0.139:11311
export ROS_HOSTNAME=192.168.0.171
source ~/catkin_ws/devel/setup.bash

##### robochair
export ROS_MASTER_URI=http://192.168.0.139:11311
export ROS_IP=192.168.0.171  # Your local IP
source devel/setup.bash
roslaunch kortex_driver kortex_driver.launch gripper:=robotiq_2f_85 vision:=false

## 2. download and build ros workspace
Do this on all agent computers

cd ~/catkin_ws
catkin_make
source devel/setup.bash

## 3. run the task with wheelchair + kinova (robochair) and patient lift (hoyer_sling)

##### robochair

###### a) wheelchair and kinova perception
roslaunch ros_chariot_master robochair_perception.launch

Notes: 
kinova wrist realsense d435i

 device info 
Device Name: Intel RealSense D435I
Device Serial No: 036522072362

 if only one camera connected #####
roslaunch realsense2_camera rs_camera.launch enable_depth:=false camera:=kinova_wrist

 if 2 cameras connected #####
roslaunch realsense2_camera rs_camera.launch \
  serial_no:=036522072362 \
  enable_depth:=false \
  camera:=kinova_wrist

 
###### b) kinova driver
roslaunch kortex_driver kortex_driver.launch gripper:=robotiq_2f_85 vision:=false
rviz -> recent congif -> kinova_visualization

 
##### compressed image topic #####
rosrun image_transport republish raw in:=/kinova_wrist/color/image_raw compressed out:=/kinova_wrist/color/image_compressed_master




3. attach image onto the robot (camera_link = d435i_link)

rosrun tf2_ros static_transform_publisher 0 0 0 0 -1.5708 -1.5708 d435i_link kinova_wrist_link


4. aruco marker detection & add to the tf

~/catkin_ws/src/robochair/src/chariot_aruco_marker$ python3 aruco_pose_est.py

5. aruco_marker_frame to bask_link lookup (don't need to run separately)

aruco_base_trans.py
rosrun tf tf_echo base_link tool_frame


6. robot control 
kinova_grasp_strap.py
