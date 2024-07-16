#! /bin/bash

PREFIX=$(date +%Y-%m-%d-%H-%M-%S)

BAGS=$HOME/robotx-2022/bags/$(date +%Y_%m%d_%H%M)

if [ ! -d "$BAGS" ]; then
    mkdir -m 775 $BAGS
fi

BAGS=$BAGS"/"$PREFIX
echo "BAGS: "$BAGS

rosbag record -O $BAGS \
    /camera_left/color/image_raw/compressed \
	/camera_left/aligned_depth_to_color/image_raw/compressedDepth \
    /camera_right/color/image_raw/compressed \
	/camera_right/aligned_depth_to_color/image_raw/compressedDepth \
    /camera_mid/color/image_raw/compressed \
	/camera_mid/aligned_depth_to_color/image_raw/compressedDepth \
	/mavros/local_position/pose \
	/mavros/local_position/velocity_local \
	/mavros/global_position/compass_hdg \
	/mavros/global_position/global \
	/mavros/global_position/raw/satellites \
	/mavros/global_position/gps_vel \
	/mavros/state \
	/mavros/rc/out \
	/tf \
	/tf_static \
	/end_effector_pose \
	/wx200/joint_states \
	/joint_states_test \
	/vr_joy_drone





