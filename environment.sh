#source /opt/ros/melodic/setup.bash
source $HOME/drone_ros/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/drone_ros/Firmware
. ~/drone_ros/Firmware/Tools/setup_gazebo.bash ~/drone_ros/Firmware ~/drone_ros/Firmware/build/px4_sitl_default


source set_ros_master.sh 127.0.0.1
source set_ros_ip.sh 127.0.0.1
