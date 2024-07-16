#! /usr/bin/env python3

import fix_python3_path
import rospy
import tf.transformations as tft
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped


class ModelStateToPoseStamped:
    def __init__(self):
        rospy.init_node("model_state_to_pose_stamped")
        self.model_name = rospy.get_param("~model_name", "wamv")
        self.relative_entity_name = rospy.get_param("~relative_entity_name", "world")  # default to "world"
        self.publish_rate = rospy.get_param("~publish_rate", 20)

        self.pose_msg = PoseStamped()
        
        self.pose_publisher = rospy.Publisher(f"/gazebo/{self.model_name}/pose", PoseStamped, queue_size=10)
        self.model_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback)
        self.pub_timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_pose)


    def model_callback(self, msg):
        try:
            model_index = msg.name.index(self.model_name)
            rospy.loginfo_throttle(1, f"Model name: {msg.name[model_index]}")

            # Find the relative entity's index
            if self.relative_entity_name and self.relative_entity_name != "world":
                if self.relative_entity_name in msg.name:
                    relative_entity_index = msg.name.index(self.relative_entity_name)
                else:
                    rospy.logerr_throttle(1, f"Relative entity {self.relative_entity_name} not found.")
                    return

                # Compute the relative pose
                model_pose = msg.pose[model_index]
                relative_entity_pose = msg.pose[relative_entity_index]
                relative_pose = self.compute_relative_pose(model_pose, relative_entity_pose)

                # Update the PoseStamped message
                self.pose_msg.header.stamp = rospy.Time.now()
                self.pose_msg.header.frame_id = self.relative_entity_name + "/base_link"
                self.pose_msg.pose = relative_pose
            else:
                model_pose = msg.pose[model_index]
                self.pose_msg.header.stamp = rospy.Time.now()
                self.pose_msg.header.frame_id = "map"
                self.pose_msg.pose = model_pose
        except ValueError:
            rospy.logerr_throttle(1, f"Model with name {self.model_name} not found in the model states.")

    def compute_relative_pose(self, model_pose, relative_entity_pose):
        # Calculate the relative transformation
        model_quaternion = [
            model_pose.orientation.x,
            model_pose.orientation.y,
            model_pose.orientation.z,
            model_pose.orientation.w,
        ]
        relative_entity_quaternion = [
            relative_entity_pose.orientation.x,
            relative_entity_pose.orientation.y,
            relative_entity_pose.orientation.z,
            relative_entity_pose.orientation.w,
        ]

        relative_rotation = tft.quaternion_multiply(
            tft.quaternion_inverse(relative_entity_quaternion), model_quaternion
        )
        relative_position = tft.quaternion_multiply(
            tft.quaternion_multiply(
                tft.quaternion_inverse(relative_entity_quaternion),
                [
                    model_pose.position.x - relative_entity_pose.position.x,
                    model_pose.position.y - relative_entity_pose.position.y,
                    model_pose.position.z - relative_entity_pose.position.z,
                    0,
                ],
            ),
            relative_entity_quaternion,
        )[:3]

        # Convert to Pose
        relative_pose = model_pose
        relative_pose.position.x, relative_pose.position.y, relative_pose.position.z = relative_position
        (
            relative_pose.orientation.x,
            relative_pose.orientation.y,
            relative_pose.orientation.z,
            relative_pose.orientation.w,
        ) = relative_rotation

        return relative_pose

    def publish_pose(self, event):
        rospy.loginfo_throttle(1, f"Publishing pose for {self.model_name}")
        self.pose_publisher.publish(self.pose_msg)


if __name__ == "__main__":
    ModelStateToPoseStamped()
    rospy.spin()
