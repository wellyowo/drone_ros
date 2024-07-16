#! /bin/bash

PREFIX=$(date +%Y-%m-%d-%H-%M-%S)

BAGS=$HOME/robotx-2022/bags/$(date +%Y_%m%d_%H%M)

if [ ! -d "$BAGS" ]; then
    mkdir -m 775 $BAGS
fi

BAGS=$BAGS"/"$PREFIX
echo "BAGS: "$BAGS

rosbag record -O $BAGS \
    /drone/downward_camera/rgb/image_raw/compressed \
	/drone/downward_camera/depth/image_raw/compressedDepth \
	/drone/front_camera/rgb/image_raw/compressed \
	/drone/front_camera/depth/image_raw/compressedDepth \
	/drone/left_camera/rgb/image_raw/compressed \
	/drone/left_camera/depth/image_raw/compressedDepth \
	/drone/right_camera/rgb/image_raw/compressed \
	/drone/right_camera/depth/image_raw/compressedDepth \
	/mavros/local_position/pose \
	/mavros/local_position/velocity_local \
	/mavros/global_position/compass_hdg \
	/mavros/global_position/global \
	/mavros/global_position/raw/satellites \
	/mavros/global_position/gps_vel \
	/mavros/state \
	/tf \
	/tf_static \
	/end_effector_pose \
	/wx200/joint_states \
	/wx200/joint_states_fix \
	/joint_states_test \
	/gazebo/model_states \
	/gazebo/drone/pose \
	/gazebo/target/pose \
	/vr_joy_drone \
	/checker \
	/front_camera/target/pose/drone_baselink \
	/drone_waypoint/visual/local \
	





