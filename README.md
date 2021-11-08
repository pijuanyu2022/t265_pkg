# t265_pkg

quickstart

step1: roslaunch realsense2_camera rs_t265.launch

step2: roslaunch test_pkg t265_fisheye_undistort.launch

step3: roslaunch pointcloud_to_laserscan start.launch

step4: roslaunch costmap_2d laser.launch

step5: rviz

---

NEW SEP 24th


Step1: source ~/catkin_navigation/devel/setup.bash

Step2: roslaunch realsense2_camera rs_t265.launch

Step3: source ~/catkin_navigation/devel_isolated/setup.bash

Step4: roslaunch test_pkg t265_fisheye_undistort.launch

Step5: source ~/catkin_navigation/devel/setup.bash

Step6: roslaunch octomap_server octomap_mapping.launch

Step7: source ~/catkin_navigation/devel/setup.bash

Step8: rosrun map_server map_saver --occ 90 --free 10 -f mymap2 map:=/projected_map



Simulation:

Step1: 

       source ~/catkin_slam/devel/setup.bash
       
       export TURTLEBOT3_MODEL=burger

       roslaunch turtlebot3_gazebo turtlebot3_world.launch

Step2: 

       source ~/catkin_slam/devel/setup.bash

       export TURTLEBOT3_MODEL=burger

       roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map1.yaml

Step3: 

       source ~/catkin_slam/devel/setup.bash

       export TURTLEBOT3_MODEL=burger

       roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
       
       
       xhost +
       ssh omnid@omnid3 -Y


Method 1:

       source ~/catkin_test/install/setup.bash

       roslaunch omnid_control mobile_interface.launch robot_id:=3

       ssh omnid@omnid3
       
       source ~/catkin_ws/devel/setup.bash
       
       roslaunch realsense2_camera rs_t265.launch
       
       source ~/catkin_test/install/setup.bash
       
       roslaunch test_pkg no_t265.launch
       
chronyc:

systemctl restart chronyd

sudo chronyc clients

sudo chronyc sources

chronyc tracking

## chronyc tutorial:

### primary computer:

sudo apt update; sudo apt install -y build-essential net-tools curl git software-properties-common;

sudo apt update; sudo apt install chrony -y;

sudo systemctl start chrony; sudo systemctl enable chrony;

nano /etc/chrony/chrony.conf

       pool 0.jp.pool.ntp.org iburst maxsources 2

       pool 1.jp.pool.ntp.org iburst maxsources 2

       pool 2.jp.pool.ntp.org iburst maxsources 2

       pool 3.jp.pool.ntp.org iburst maxsources 2

       allow 192.168.1.50 (depend on your computer)
       
       allow 192.168.1.50/24
       
sudo systemctl restart chrony
sudo chronyc clients


### second computer:

sudo apt update; sudo apt install chrony -y;

sudo systemctl start chrony; sudo systemctl enable chrony;

nano /etc/chrony/chrony.conf

       server 192.168.1.50 (depend on your computer) prefer iburst

sudo systemctl restart chrony

sudo chronyc sources

sudo chronyc sources -v
