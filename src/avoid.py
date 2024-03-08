#!/usr/bin/env python3
# ROS Noetic
# Node Name: avoid
# Subscribed Topics: /scan, /mv_cmd
# Published Topics: /cmd_vel
#  
# Given a Twist from topic /mv_cmd and laser scan from /scan
# remove the linear velocity if the robot is going to hit some obstacles
# some times after this instant
# How the robot is going to determine this is through the defined constants and laser scan
# see config/config.yaml to see what parameters are expected


import rospy, math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray
from utils import findDistance, degree_stdize
import numpy as np
from copy import deepcopy 



# constant
DIAMETER = rospy.get_param("DIAMETER") # m
DISTANCE_TOLERANCE = rospy.get_param("DISTANCE_TOLERANCE") # m
FORESIGHT_TIMESTAMPS = rospy.get_param("FORESIGHT_TIMESTAMPS") # [<seconds>, <seconds>]
SAMPLE_ANGLE = rospy.get_param("SAMPLE_ANGLE") # degree

class AvoidBot:
    def __init__(self, 
            diameter=DIAMETER, 
            distance_tolrance=DISTANCE_TOLERANCE, 
            foresight_timestamps=FORESIGHT_TIMESTAMPS, 
            sample_angle=SAMPLE_ANGLE):
        # setup
        self.diameter = diameter
        self.distance_tolerance = distance_tolrance
        self.ang_samples = [0, *range(1, sample_angle + 1), *range(-1, -sample_angle - 1, -1)]
        self.foresight_timestamps = foresight_timestamps

        # subscribed
        self.x, self.y  = None, None
        self.distance_at_angle = [0] * 360
        self.mv_cmd_twist = None

        self.init_laser = False

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def laser_update(self, laser_read):
        angles = range(0, 360, 1)
        for angle in angles:
            self.distance_at_angle[angle] = float("inf") if laser_read.ranges[angle] == 0 else  laser_read.ranges[angle]
        self.init_laser = True

    def mv_cmd_twist_update(self, twist_msg):
        self.mv_cmd_twist = twist_msg # reactive of follow
        self.pub.publish(self.foresight_twist(self.foresight_timestamps))


    def foresight_obtacles(self, foresight_timestamps):
        if self.mv_cmd_twist is None:
            return False # assuming no movement, no hitting

        v0, delta_theta = self.mv_cmd_twist.linear.x, self.mv_cmd_twist.angular.z
        for timestamp in foresight_timestamps:
            if delta_theta != 0:
                x = v0 * math.cos(0) / delta_theta - v0 * math.cos(delta_theta * timestamp) / delta_theta
                y = v0 * math.sin(delta_theta * timestamp) / delta_theta
            else:
                x = 0
                y = v0 * timestamp

            
            theta = degree_stdize(round(math.degrees(math.atan2(y, x)))) # standardize the degree from 0 to 359
            
            # only check angles around the distance since we are moving there 
            _sample_angles = [degree_stdize(i + theta) for i in self.ang_samples]
            xs_far = [self.distance_at_angle[i] * math.sin(math.radians(i)) for i in _sample_angles]
            ys_far = [self.distance_at_angle[i] * math.cos(math.radians(i)) for i in _sample_angles]


            for i, (x1, y1) in enumerate(zip(xs_far, ys_far)):
                d = findDistance(x, y, x1, y1)
                # print("Distance:", d) 

                
                if d <= self.distance_tolerance:
                    # print("==================================")
                    # print("Obstacles Forsighted")
                    # print("Angle:", i)
                    # print("Cur Twist:", (v0, delta_theta))
                    # print("Distance:", d)
                    # print(f"In: {timestamp} s")
                    # print(f"Forsighted Location:", (x, y))
                    # print(f"Obstacles Location:", (x1, y1))
                    # print("==================================")  
                    return True
        return False

    def foresight_twist(self, foresight_timestamps):
    
        if self.mv_cmd_twist is None:
            return None # assuming no movement, no hitting

        res = deepcopy(self.mv_cmd_twist)
        
        if self.foresight_obtacles(foresight_timestamps):
            # remove all linear speed to avoid hitting things
            # still turns towards the target if possible
            # if the laser is covered, it is likely that the camera is covered as well
            res.linear.x = 0
            res.linear.y = 0
            res.linear.z = 0
 
        return res


if __name__ == '__main__':

    # setup
    rob = AvoidBot()
    rospy.init_node('avoid', anonymous=True)


    rospy.Subscriber("/scan", LaserScan, rob.laser_update)   
    rospy.Subscriber("/mv_cmd", Twist, rob.mv_cmd_twist_update) # subscribed to the commanded twist

    # wait for lidar to response every 3 sec
    wait_period = 3 # ss
    while not rospy.is_shutdown() and (not rob.init_laser):
        print("wait for sensors to response")
        rospy.sleep(wait_period)
    print("Sensors responded")

    rospy.spin()
