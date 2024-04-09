#!/usr/bin/env python3

# create a ros node to receive the goal pose
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult
from sensor_msgs.msg import Image, LaserScan
import cv2
import numpy as np
import tf
import math


class goal:
    def __init__(self):
        self.finding_path = [[8, 0], [8, -6], [16, -3], [16, 0]]
        self.find_path_idx = 0
        self.find_box = "free"
        self.wait_for_reach = False
        self.image_count = 0
        self.adapt_count = 0
        self.goal_pose = [0, 0]
        rospy.init_node('goal_publisher', anonymous=True)
        pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        image_path = '/home/marmot/dixiao/ME5413_Final_Project/src/me5413_world/scripts/4.jpg'
        self.image = cv2.imread(image_path)
        scale = 1
        self.image_list = [cv2.resize(self.image, (0, 0), fx=scale * i / 10, fy=scale * i / 10) for i in range(2, 10)]

        rospy.Subscriber('/rviz_panel/goal_name', String, self.callback_first_start, pub)
        rospy.Subscriber('/front/image_raw', Image, self.callback_image, pub)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback_pose, pub)
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.callback_goal)
        rospy.Subscriber('/front/scan', LaserScan, self.callback_scan)

        rospy.spin()

    def callback_scan(self, data):

        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_increment = data.angle_increment
        ranges = data.ranges

        front_angle = 0.0
        relative_angle = front_angle - angle_min

        idx = int(relative_angle / angle_increment)

        if idx >= 0 and idx <= len(ranges):
            distance = ranges[idx]
            self.distance = distance

    def callback_goal(self, data):
        self.goal_pose = [data.pose.position.x, data.pose.position.y]

    def callback_first_start(self, data, pub):
        if data.data[1:4] == 'box':
            self.find_box = "box"
            self.box_idx = int(data.data[-1])
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = 8
            goal_pose.pose.position.y = 0.0
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.x = 0.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = 0.0
            goal_pose.pose.orientation.w = 1.0

            pub.publish(goal_pose)

    def callback_pose(self, data, pub):
        # tf to get the robot pose in map frame

        self.pose = data.pose.pose
        self.orin = data.pose.pose.orientation
        robot_pose = [self.pose.position.x, self.pose.position.y]
        goal_pose = self.finding_path[self.find_path_idx]
        distance = np.sqrt((robot_pose[0] - goal_pose[0]) ** 2 + (robot_pose[1] - goal_pose[1]) ** 2)
        if distance < 0.5 and self.find_box == "box":
            self.find_path_idx += 1
            self.find_path_idx = self.find_path_idx % len(self.finding_path)

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = self.finding_path[self.find_path_idx][0]
            goal_pose.pose.position.y = self.finding_path[self.find_path_idx][1]
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.x = 0.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = 0.0
            goal_pose.pose.orientation.w = 1.0
            self.goal_pose = [goal_pose.pose.position.x, goal_pose.pose.position.y]

            pub.publish(goal_pose)

    def callback_image(self, data, pub):
        # use opencv to show the image
        # change an Image to numpy
        height, width, channels = data.height, data.width, data.step
        frame = np.frombuffer(data.data, dtype=np.uint8).reshape(height, width, -1)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if self.find_box == 'go_box':
            self.image_count += 1
            self.image_count = self.image_count % 60
            if self.image_count == 0:
                self.wait_for_reach = False
        for image in self.image_list:
            res = cv2.matchTemplate(frame, image, cv2.TM_CCOEFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
            threshold = 0.7

            if max_val > threshold and (
                    self.find_box == 'box' or self.find_box == 'go_box') and not self.wait_for_reach:
                print('Found box')
                self.find_box = 'go_box'
                cv2.rectangle(frame, max_loc, (max_loc[0] + image.shape[1], max_loc[1] + image.shape[0]), (0, 255, 0),
                              10)
                cv2.imshow('frame', frame)
                cv2.waitKey(3)
                # location of the box in map frame
                eular = tf.transformations.euler_from_quaternion(
                    [self.orin.x, self.orin.y, self.orin.z, self.orin.w])
                yaw = eular[2]

                image_center_x = width / 2
                target_x = max_loc[0] + image.shape[1] / 2
                offset_x = target_x - image_center_x
                angle_offset = (offset_x / width) / 1.0471975512

                target_orientation = yaw - angle_offset
                if target_orientation > math.pi:
                    target_orientation -= 2 * math.pi
                if target_orientation < -math.pi:
                    target_orientation += 2 * math.pi

                box_pose = PoseStamped()
                box_pose.header.frame_id = 'map'
                box_pose.pose.position.x = self.pose.position.x + 0.1 * math.cos(target_orientation)
                box_pose.pose.position.y = self.pose.position.y + 0.1 * math.sin(target_orientation)
                self.goal_pose = [box_pose.pose.position.x, box_pose.pose.position.y]
                box_pose.pose.position.z = 0.0
                quaternion = tf.transformations.quaternion_from_euler(0, 0, target_orientation)
                box_pose.pose.orientation.x = quaternion[0]
                box_pose.pose.orientation.y = quaternion[1]
                box_pose.pose.orientation.z = quaternion[2]
                box_pose.pose.orientation.w = quaternion[3]
                self.wait_for_reach = True
                self.adapt_count += 1
                if self.adapt_count == 3:
                    self.adapt_count = self.adapt_count % 3
                    box_pose.pose.position.x = self.pose.position.x + (self.distance - 0.25) * math.cos(target_orientation)
                    box_pose.pose.position.y = self.pose.position.y + (self.distance - 0.25
                                                                       ) * math.sin(target_orientation)
                    self.find_box = 'go_to_box'
                    print('Go to box')
                pub.publish(box_pose)
                break


if __name__ == '__main__':
    goal()
