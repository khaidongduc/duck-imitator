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
        self.cur_twist = None
        self.heading = None

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
    
        self.front_dis = min(self.distance_at_angle[angle] * math.cos(math.radians(abs(angle))) 
                             for angle in self.front_dis_ang_samples)
        self.init_laser = True

    def twist_update(self, twist_msg):
        self.cur_twist = twist_msg

    def heading_update(self, heading_msg):
        self.heading = heading_msg

    def create_adjusted_twist(self, max_lin_speed=0.3, max_ang_spd=math.radians(20)):
        out_msg = Twist()

        if self.heading is None: # spin around if no heading is given
            out_msg.angular.z = max_ang_spd / 2
            # possibly found a heading if keep turning around

        return out_msg


if __name__ == '__main__':

    # constant
    diameter = 0.07 # m
    distance_tolerance = 0.5 # m
    linear_spd, angular_speed = 0.2, math.radians(30) # m/s, rad/s
    safe_distance = 0.5 # m

    # setup
    rob = AvoidObstTurtleBot(diameter, distance_tolerance)
    rospy.init_node('avoid', anonymous=True)



    rospy.Subscriber("/odom", Odometry, rob.odom_update)
    rospy.Subscriber("/scan", LaserScan, rob.laser_update)   

    rospy.Subscriber("/cmd_vel", Twist, rob.twist_update) # to slowly ease out from current speed for smoother   
    rospy.Subscriber("/heading", Odometry, rob.heading_update) # to know the desired heading

    rate = rospy.Rate(10)

    # wait for odometer and lidar to response
    while not rob.init_laser or not rob.init_odom:
        print("wait for sensors to response")
        rate.sleep()
    print("Sensors responded")

    
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    while not rospy.is_shutdown():
        msg = rob.create_adjusted_twist()
        pub.publish(msg)
        rate.sleep()

    rospy.spin()
