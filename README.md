# t265_pkg

quickstart

step1: roslaunch realsense2_camera rs_t265.launch

step2: roslaunch test_pkg t265_fisheye_undistort.launch

step3: roslaunch pointcloud_to_laserscan start.launch

step4: roslaunch costmap_2d laser.launch

step5: rviz

---

NEW SEP 24th


Step1: source ~/catkin_real/devel/setup.bash

Step2: roslaunch realsense2_camera rs_t265.launch

Step3: source ~/catkin_april/devel_isolated/setup.bash

Step4: roslaunch test_pkg t265_fisheye_undistort.launch

Step5: source ~/catkin_navi/devel/setup.bash

Step6: roslaunch octomap_server octomap_mapping.launch

Step7: source ~/catkin_navi/devel/setup.bash

Step8: rosrun map_server map_saver --occ 90 --free 10 -f mymap2 map:=/projected_map



Simulation:

Step1: source ~/catkin_slam/devel/setup.bash
       
       export TURTLEBOT3_MODEL=burger

       roslaunch turtlebot3_gazebo turtlebot3_world.launch

Step2: source ~/catkin_slam/devel/setup.bash

       export TURTLEBOT3_MODEL=burger

       roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map1.yaml

Step3: source ~/catkin_slam/devel/setup.bash

       export TURTLEBOT3_MODEL=burger

       roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

