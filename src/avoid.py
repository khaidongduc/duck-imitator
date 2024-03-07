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


# constant
diameter = 0.07 # m
theta_tolerance = 0.2 # rad
linear_spd, angular_speed = 0.2, math.radians(30) # m/s, rad/s
safe_distance = 0.3 # m
follow_distance = 0.5 # m
foresight_timestamps = np.arange(0, 0.5, 0.1) #s


class AvoidObstTurtleBot:
    def __init__(self, diameter, distance_tolrance):
           # setup
        self.diameter = diameter
        self.distance_tolerance = distance_tolrance
        
        sample_angle = math.ceil(math.degrees(math.atan(self.diameter / self.distance_tolerance)))
        self.ang_samples = [0, *range(1, sample_angle + 1), *range(-1, -sample_angle - 1, -1)]

        # subscribed
        self.x, self.y  = None, None
        self.distance_at_angle = [0] * 360
        self.roll, self.pitch, self.yaw = None, None, None
        self.cur_twist = None
        self.heading = None
        self.last_valid_heading = None

        self.init_odom = False
        self.init_laser = False

    def odom_update(self, odom_read):
        self.x, self.y = odom_read.pose.pose.position.x, odom_read.pose.pose.position.y
        orient = odom_read.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion((orient.x, orient.y, orient.z, orient.w))
        self.init_odom = True

    def laser_update(self, laser_read):
        angles = range(0, 360, 1)
        for angle in angles:
            self.distance_at_angle[angle] = float("inf") if laser_read.ranges[angle] == 0 else  laser_read.ranges[angle]
        self.init_laser = True

    def twist_update(self, twist_msg):
        self.cur_twist = twist_msg

    def heading_update(self, heading_msg):
        self.heading = heading_msg
        if len(self.heading.data) == 2: # valid
            self.last_valid_heading = heading_msg

    def create_adjusted_twist(self, 
            follow_distance=0.5, # m
            theta_tolerance = 0.2, # rad 
            max_lin_speed=0.3, max_ang_spd=math.radians(30)):
        
        out_msg = Twist()

        if self.heading is None or len(self.heading.data) == 0: # spin around if no heading is given
            out_msg.angular.z = max_ang_spd
            if self.last_valid_heading is not None:
                last_theta = self.last_valid_heading.data[1]
                # print("last_theta:", last_theta)
                out_msg.angular.z *= last_theta / abs(last_theta)
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
    
        return out_msg

    def foresight_obtacles(self, foresight_timestamps):
        if self.cur_twist is None:
            return False # assuming no movement, no hitting

        v0, delta_theta = self.cur_twist.linear.x, self.cur_twist.angular.z
    
        for timestamp in foresight_timestamps:
            if delta_theta != 0:
                x = v0 * math.cos(0) / delta_theta - v0 * math.cos(delta_theta * timestamp) / delta_theta
                y = v0 * math.sin(delta_theta * timestamp) / delta_theta
            else:
                x = 0
                y = v0 * timestamp


            degree_stdize = lambda x: (x + 360) % 360
            theta = (round(math.degrees(math.atan2(y, x))) + 360) % 360 # standardize the degree from 0 to 359
            
            # only check angles around the distance since we are moving there 
            _sample_angles = [degree_stdize(i + theta) for i in self.ang_samples]
            xs_far = [self.distance_at_angle[i] * math.cos(math.radians(i)) for i in _sample_angles]
            ys_far = [self.distance_at_angle[i] * math.sin(math.radians(i)) for i in _sample_angles]

            for x1, y1 in zip(xs_far, ys_far):
                d = findDistance(x, y, x1, y1)
                # print("Distance:", d) 
                if d <= self.distance_tolerance:
                    return True
        return False

    def foresight_twist(self):
        return Twist()

if __name__ == '__main__':

    # setup
    rob = AvoidObstTurtleBot(diameter, safe_distance)
    rospy.init_node('avoid', anonymous=True)



    # rospy.Subscriber("/odom", Odometry, rob.odom_update)
    rospy.Subscriber("/scan", LaserScan, rob.laser_update)   

    rospy.Subscriber("/cmd_vel", Twist, rob.twist_update) # to slowly ease out from current speed for smoother   
    rospy.Subscriber("/heading", Float32MultiArray, rob.heading_update) # to know the desired heading



    
    # wait for lidar to response
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown() and (not rob.init_laser):
        print("wait for sensors to response")
        rate.sleep()
    print("Sensors responded")

    rate = rospy.Rate(10)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    while not rospy.is_shutdown():
        about_to_hit = rob.foresight_obtacles(foresight_timestamps=foresight_timestamps)
        print(about_to_hit)
        if about_to_hit:
            msg = rob.foresight_twist()
        else:
            msg = rob.create_adjusted_twist(
                theta_tolerance=theta_tolerance,
                follow_distance=follow_distance)
        print(msg)
        pub.publish(msg)
        rate.sleep()

    rospy.spin()
