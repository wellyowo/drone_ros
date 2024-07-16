touch ~/drone_ros/catkin_ws/src/geometry2/CATKIN_IGNORE
catkin build -w ~/drone_ros/catkin_ws

cd catkin_ws && rosdep install --from-paths src --ignore-src -y -r

rm ~/drone_ros/catkin_ws/src/geometry2/CATKIN_IGNORE
catkin build -w ~/drone_ros/catkin_ws geometry2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
touch ~/drone_ros/catkin_ws/src/geometry2/CATKIN_IGNORE

git config --global --add safe.directory /home/argrobotx/drone_ros/Firmware
git config --global --add safe.directory /home/aspawn_model_pillar2-36rgrobotx/drone_ros/Firmware/src/modules/mavlink/mavlink
git config --global --add safe.directory /home/argrobotx/drone_ros/Firmware/platforms/nuttx/NuttX/nuttx

cd ..
cd Firmware && ./Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx
DONT_RUN=1 make px4_sitl_default gazebo
cd ..
