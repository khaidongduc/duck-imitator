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

class AvoidObstTurtleBot:
    def __init__(self, diameter, distance_tolrance):
           # setup
        self.diameter = diameter
        self.distance_tolerance = distance_tolrance
        self.front_dis_ang_samples = [0]
        sample_angle = math.ceil(math.degrees(math.atan(self.diameter / self.distance_tolerance)))
        self.front_dis_ang_samples = [0, *range(1, sample_angle + 1), *range(-1, -sample_angle - 1, -1)]

        # subscribed
        self.x, self.y  = None, None
        self.distance_at_angle = [0] * 360
        self.roll, self.pitch, self.yaw = None, None, None
        self.front_dis = None
        self.init_odom = False
        self.init_laser = False
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def odom_update(self, odom_read):
        self.x, self.y = odom_read.pose.pose.position.x, odom_read.pose.pose.position.y
        orient = odom_read.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion((orient.x, orient.y, orient.z, orient.w))
        self.init_odom = True

    def laser_update(self, laser_read):
        angles = range(0, 360, 1)
        for angle in angles:
            self.distance_at_angle[angle] = float("inf") if laser_read.ranges[angle] == 0 else  laser_read.ranges[angle]
    
        self.front_dis = min(self.distance_at_angle[angle] * math.cos(math.radians(abs(angle))) 
                             for angle in self.front_dis_ang_samples)
        self.init_laser = True

    def mv_forward(self, lin_spd):
        forward_msg = Twist()
        forward_msg.linear.x = lin_spd
        self.pub.publish(forward_msg)

    def stop(self):
        self.pub.publish(Twist())

    def rotate(self, angle, max_ang_spd, isClockwise, tolerance=math.radians(0.1)):
        # use odometry data to figure out the current angle instead of timing
        outData = Twist()   
        initial_degree = self.yaw
        target_degree = self.yaw + (-1)**isClockwise * angle
        if target_degree < 0:
            target_degree += 2 * math.pi

        min_spd = tolerance / 0.1 # magic number, assuming the robot can stop within 0.1 second

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            cur_degree = self.yaw
            if cur_degree < 0:
                cur_degree += 2 * math.pi
            if  abs(cur_degree - target_degree) <= tolerance:
                break
            outData.angular.z = (-1)**isClockwise * max_ang_spd \
                        * math.sqrt(abs(cur_degree - target_degree) / abs(initial_degree - target_degree))
            if abs(outData.angular.z) < min_spd:
                outData.angular.z = outData.angular.z / abs(outData.angular.z) * min_spd
            self.pub.publish(outData)
            rate.sleep()
        # print()
        self.pub.publish(Twist())   

    def angle_with_most_free_space(self):
        # make a copy before calculation to avoid changing
        lidar_reading = self.distance_at_angle.copy()
        res_angle, max_dist = -1, 0
        #  take of 45 degree positions
        angles = range(0, 360, 45)
        for angle in angles:
            dist_at_angle = min(lidar_reading[(angle + a + 360) % 360] * math.cos(math.radians(abs(a))) 
                                 for a in self.front_dis_ang_samples)    
            print("Angle, Distance: ", angle, dist_at_angle)
            print([lidar_reading[(angle + a + 360) % 360] * math.cos(math.radians(abs(a))) for a in self.front_dis_ang_samples])

            if dist_at_angle > max_dist:
                max_dist = dist_at_angle
                res_angle = angle
        print()

        return res_angle

if __name__ == '__main__':

    # constant
    diameter = 0.07 # m
    distance_tolerance = 0.5 # m
    linear_spd, angular_speed = 0.2, math.radians(30) # m/s, rad/s
    safe_distance = 0.5 # m

    # setup
    rob = AvoidObstTurtleBot(diameter, distance_tolerance)
    rospy.init_node('avoid_obstacles', anonymous=True)
    rospy.Subscriber("/odom", Odometry, rob.odom_update)
    rospy.Subscriber("/scan", LaserScan, rob.laser_update)    
    rate = rospy.Rate(10)

    # wait for odometer and lidar to response
    while not rob.init_laser or not rob.init_odom:
        print("wait for sensors to response")
        rate.sleep()
    print("Sensors responded")

    # for angle in range(0, 360, 45):
    #     input(f"{angle}")
    #     rob.rotate(math.radians(angle), angular_speed, True)

    while not rospy.is_shutdown():
        front_dis = rob.front_dis
        if front_dis > rob.distance_tolerance:
            print("No ostacles ahead, move forward")
            rob.mv_forward(linear_spd)
        else:
            rob.stop()
            # turn to the angle with the most free space
            angle = rob.angle_with_most_free_space()
            print("Ostacles ahead, turn to angle with most space")
            print("angle w/ most free space: ", angle)
            rob.rotate(math.radians(angle), angular_speed, True)
            
        rate.sleep()

    rospy.spin()
