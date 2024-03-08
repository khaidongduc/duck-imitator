#!/usr/bin/env python3
# Edited by Khai Dong (dongk@union.edu)

# behavior description:
# the robot will move forward if there is no obstacles 0.5m ahead
# since the robot is not a single point, aside from 0 degree right ahead, the robot also samples
# some ajacent angles to see if its sides may hit an object
# if there are objects are head, the robot will choose the angle with the most free space
# and turn to that angle
# the angle choice has the same mechanism as it will samples nearby angles
# there is some trignometry involved to figured how much ajacent angles to be sampled
# in rotating, I use the odometry data instead of timing for a more accurate turn
#               moreover, I slow the robot down as it is approaching the desired location



import rospy, math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray
from utils import findDistance
import numpy as np
import os

# constant
THETA_TOLERANCE = rospy.get_param("THETA_TOLERANCE") # rad
LINEAR_SPEED = rospy.get_param("LINEAR_SPEED") # m/s, 
ANGULAR_SPEED = math.radians(rospy.get_param("ANGULAR_SPEED")) # rad/s
FOLLOW_DISTANCE = rospy.get_param("FOLLOW_DISTANCE") # m


class FollowBot:
    def __init__(self, follow_tolerance=FOLLOW_DISTANCE):
        self.follow_tolerance = follow_tolerance
        self.heading = None
        self.last_valid_heading = None


    def heading_update(self, heading_msg):
        self.heading = heading_msg
        if len(self.heading.data) == 2: # valid
            self.last_valid_heading = heading_msg

    def create_adjusted_twist(self,
            theta_tolerance=THETA_TOLERANCE, # rad 
            max_lin_speed=LINEAR_SPEED, max_ang_spd=ANGULAR_SPEED):
        
        out_msg = Twist()

        if self.heading is None or len(self.heading.data) == 0: # spin around if no heading is given
            # out_msg.angular.z = max_ang_spd
            # if self.last_valid_heading is not None:
            #     last_theta = self.last_valid_heading.data[1]
            #     # print("last_theta:", last_theta)
            #     out_msg.angular.z *= last_theta / abs(last_theta)
            return out_msg

        theta = self.heading.data[1]
        d = self.heading.data[0] # m
        if d <= 0:
            return out_msg    
        if abs(2 * math.pi - theta) < abs(theta):
            theta = - (2 * math.pi - theta)

        angular_spd_control = min(1, abs(theta) / (max_ang_spd)) # priorize rotating when theta is large
        lin_spd_control = 1 - angular_spd_control

        if abs(theta) > theta_tolerance:
            out_msg.angular.z = angular_spd_control * theta * max_ang_spd
            # make sure angular speed don't exceed max
            if abs(out_msg.angular.z) >= max_ang_spd:
                out_msg.angular.z = out_msg.angular.z / abs(out_msg.angular.z) * max_ang_spd
        if d > follow_distance:
            out_msg.linear.x = max(max_lin_speed, lin_spd_control * d * max_lin_speed)
            # return out_msg

        # insert spinning direction in case of inverted camera
        out_msg.angular.z = - out_msg.angular.z 
        
        return out_msg


if __name__ == '__main__':

    # setup
    rob = FollowBot()
    rospy.init_node('follow', anonymous=True)

    rospy.Subscriber("/heading", Float32MultiArray, rob.heading_update) # to know the desired heading

    rate = rospy.Rate(10)
    pub = rospy.Publisher("/mv_cmd", Twist, queue_size=10)
    while not rospy.is_shutdown():
        msg = rob.create_adjusted_twist()

        os.system('clear')
        print("follow ===================================================")
        print("(d, theta)", rob.heading)
        
        
        print(msg)
        print("==========================================================")

        pub.publish(msg)
        rate.sleep()

    rospy.spin()
