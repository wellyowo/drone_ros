#! /usr/bin/env python3
import os
import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan, Joy
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R
import yaml
import csv
import time


class Bag_to_csv(object):
    def __init__(self):
        super().__init__()
        self.record_yaml = False
        self.record_list = []
        #0.5
        self.auto = 0
        self.goal = None

        self.pose_x = None
        self.pose_y = None
        self.dis_to_goal = None
        self.angle_to_goal = None
        self.time = None
        self.linear = None
        self.angular = None
        self.start_time = None
        self.inferece_time = 0


        # save param
        self.save_param_file = '/home/argrobotx/robotx-2022/catkin_ws/src/wamv_rl/csv/bag_xxx.csv'
        self.save_csv_init()


        self.sub_joy = rospy.Subscriber("/wamv/joy", Joy, self.cb_joy, queue_size=1)
        self.sub_goal = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.cb_goal, queue_size=1)
        self.sub_odom = rospy.Subscriber(
            "/wamv/localization_gps_imu/pose", PoseStamped, self.cb_odom, queue_size=1)
        self.sub_cmd_vel = rospy.Subscriber(
            "/wamv/cmd_vel", Twist, self.cb_cmd_vel, queue_size=1)
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.inference)

    def cb_cmd_vel(self, msg):
        if self.auto == 0:
            return
        self.linear = msg.linear.x
        self.angular = msg.angular.z

    def save_csv_init(self):
        with open(self.save_param_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(['X', 'Y', 'dis_to_goal', 'angle_to_goal', 't', 'linear', 'angular'])
  
    def save_csv(self):
        with open(self.save_param_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([self.pose_x, self.pose_y, self.dis_to_goal, self.angle_to_goal, self.time, self.linear, self.angular])

    def cb_goal(self, msg):
        self.goal = np.array([
            msg.pose.position.x, 
            msg.pose.position.y,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w])

    def cb_joy(self, msg):
        start_button = 7
        back_button = 6

        if (msg.buttons[start_button] == 1) and not self.auto:
            self.auto = 1
            rospy.loginfo('go auto')
            self.start_time = time.time()

        elif msg.buttons[back_button] == 1 and self.auto:
            self.auto = 0
            rospy.loginfo('go manual')

    def cb_odom(self, msg):
        if self.goal is None:
            return

        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y
        self.time = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9

        ar = R.from_quat([msg.pose.orientation.x,
                         msg.pose.orientation.y,
                         msg.pose.orientation.z,
                         msg.pose.orientation.w])
        agent_yaw = ar.as_euler('zyx')[0]

        gr = R.from_quat([self.goal[2], 
                        self.goal[3], 
                        self.goal[4], 
                        self.goal[5]])
        goal_yaw = gr.as_euler('zyx')[0]

        angle = goal_yaw - agent_yaw 
        if angle >= np.pi:
            angle -= 2*np.pi
        elif angle <= -np.pi:
            angle += 2*np.pi


        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y
        self.dis_to_goal = math.sqrt((self.goal[0]-self.pose_x)**2+(self.goal[1]-self.pose_y )**2)
        self.angle_to_goal = angle
        self.time = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9



    def inference(self, event):
        if self.goal is None:
            print("no goal")
            return
        if self.auto == 0:
            print("auto off")
            return

        print(self.dis_to_goal)
        self.save_csv()


if __name__ == "__main__":
    rospy.init_node("bag_to_csv")
    ma_infrence = Bag_to_csv()
    rospy.spin()