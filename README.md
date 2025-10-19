In order to set-up a Gazebo simulation for the Movo,

1. Go to https://github.com/Kinovarobotics/kinova-movo/wiki/1.-Setup-Instructions and follow the manual instructions
    - When installing the additional packages under ROS, use the follow line as the Movo is using Ubuntu 16.04 with ROS Kinetic
```
sudo apt-get install ros-kinetic-control-toolbox ros-kinetic-moveit-core ros-kinetic-costmap-2d ros-kinetic-move-base ros-kinetic-jsk-recognition ros-kinetic-controller-manager ros-kinetic-gazebo-ros* ros-kinetic-hector* ros-kinetic-rviz-imu-plugin ros-kinetic-robot-pose-ekf ros-kinetic-robot-localization ros-kinetic-yocs-cmd-vel-mux ros-kinetic-joint-* libsuitesparse-dev ros-kinetic-costmap-converter ros-kinetic-libg2o ros-kinetic-eigen-conversions ros-kinetic-eigen-stl-containers libeigen3-dev ros-kinetic-trac-ik ros-kinetic-moveit ros-kinetic-ros-controllers ros-kinetic-map-server ros-kinetic-amcl ros-kinetic-global-planner libignition-math2-dev
```
    - Install MoveIt!
    - Install Gazebo7
    - Install FreeNect2, following most of the instructions except that we are using Ubuntu 16.04 instead of Ubuntu 14.04 (also take note of your cmake version)
    - Additionally, install "sudo apt-get install libignition-math2-dev"

2. Because for this project we only care about moving the robot and not controlling it's arms, I chose only particular folders from the Kinova Movo repo

3. Fix gravity bug - in Gazebo 7, there is a bug regarding gazebo_ros_control that results in gravity not being applied:
https://github.com/ros-simulation/gazebo_ros_pkgs/issues/612
    - To fix this, create a gazebo_ros_control.yaml file (as I have done in kinova-movo/movo_simulation/movo_gazebo/controller/gazebo_ros_control)
    - To find the joints to put in gazebo_ros_control.yaml, you will need to look through your URDF/xacros and find the joints with PositionJointInterface
    - You will also get some warning about PositionJointInterface that you can fix by appending "hardware_interface/" to "PositionJointInterface" 

4. To launch the Movo Gazebo simulation, do 

```
catkin_make 
roslaunch movo_gazebo movo.launch
```

Notes on using gmapping and rosbag: take note that rosbags of Movo will /map->/odom, and gmapping will also publish another /map->/odom. This results in TF multiple authority
contention. To fix this, just remap gmapping's /map to something else like /remapped. When using rviz, choose /remapped


For SLAM, use RTAB-map:
    - Take a look at the demos - http://wiki.ros.org/rtabmap_ros#Robot_mapping
    - Subscribe to /rtabmap/cloud_map and the grid_map
    - Find a way to render efficiently in Unity
