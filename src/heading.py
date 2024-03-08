#!/usr/bin/env python3

from __future__ import print_function
import roslib
import numpy as np
import sys
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image as ImageMsg
from geometry_msgs.msg import Polygon, Point32, PoseArray

from utils import get_limits, findArea
import rospy, math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from utils import findDistance

img_num_rows = rospy.get_param("img_num_rows")
img_num_cols = rospy.get_param("img_num_cols")
alpha = rospy.get_param("alpha")
beta = rospy.get_param("beta")


class Follow:
    def __init__(self):
        self.x1 = 0
        self.y1 = 0
        self.z1 = 0

        self.x2 = 0
        self.y2 = 0
        self.z2 = 0

        self.distance = 0
        self.theta_angle = 0

        self.follow_array = []
        
    def follow_callback(self, data):

        if len(data.points) == 0:
            self.follow_array = []
            # print("Distance", None)
            # print("Theta angle", None)
            return self.follow_array

        # print(data.points)

        self.x1 = data.points[0].x
        self.y1 = data.points[0].y
        self.z1 = data.points[0].z

        self.x2 = data.points[1].x
        self.y2 = data.points[1].y
        self.z2 = data.points[1].z

        img_mid_point = (img_num_cols / 2, img_num_rows / 2)
        obj_mid_point = ((self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2)
        img_obj_distance = findDistance(*obj_mid_point, *img_mid_point)

        area = findArea(self.x1, self.x2, self.y1, self.y2)

        camera_distance = 111 - math.sqrt(area) / 0.63

        self.distance = camera_distance / 2
        self.theta_angle = math.atan2(img_obj_distance, camera_distance)

        if obj_mid_point[0] - img_mid_point[0] < 0:
            self.theta_angle = -self.theta_angle

        # print("Distance", self.distance)
        # print("Theta angle", self.theta_angle)

        self.follow_array = []
        self.follow_array.append(self.distance)
        self.follow_array.append(self.theta_angle)
        # print("Array", self.follow_array)
    

if __name__ == '__main__':
    rob = Follow()
    rospy.init_node('heading', anonymous=True)

    rospy.Subscriber("point_topic", Polygon, rob.follow_callback)
    rate = rospy.Rate(10)

    pub = rospy.Publisher("/heading", Float32MultiArray, queue_size=10)
    while not rospy.is_shutdown():
        data_to_send = Float32MultiArray()
        data_to_send.data = rob.follow_array # assign the array with the value you want to send
        pub.publish(data_to_send)
        rate.sleep()
    rospy.spin()








