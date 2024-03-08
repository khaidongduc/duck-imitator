#!/usr/bin/env python3
# ROS Noetic
# Node Name: yellow_detector
# Subscribed Topics: /camera/image
# Published Topics: /image_topic, /point_topic
#  
# Given an image from topic /camera/image
# process the image to find the hull of a "yellow" square in that image
# what exactly yellow is can be specified in the config file
# publish the hull of the "yellow" square to /point_topic
# /image_topic is used for ROS Bridge to convert ROS message to OpenCV message

from __future__ import print_function
import roslib
import numpy as np
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as Img
from geometry_msgs.msg import Polygon, Point32

from utils import get_limits, findArea


THRES = rospy.get_param("THRES") 
YELLOW = rospy.get_param("YELLOW")
HUE_LIMIT = rospy.get_param("HUE_LIMIT")

class DetectBot:

    def __init__(self):
        self.image_pub = rospy.Publisher("/image_topic", ImageMsg, queue_size=10)
        self.point_pub = rospy.Publisher("/point_topic", Polygon, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image", ImageMsg, self.callback)

    def callback(self, data):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        (rows,cols,channels) = cv_image.shape


        # "Yellow" color detection
        # "yellow" in BGR colorspace
        hsvImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lowerLimit, upperLimit = get_limits(color=YELLOW, huelimit=HUE_LIMIT)
        color_mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)
        mask = color_mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find bbox in image
        mask_ = Img.fromarray(mask)
        bbox = mask_.getbbox()
        if bbox is not None:
            x1, y1, x2, y2 = bbox
            area = findArea(x1, x2, y1, y2)
            if area < THRES * rows * cols:
                bbox = None
                self.point_pub.publish(Polygon())
            else:
                line_width = 5
                cv_image = cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), line_width)
                points = Polygon()
                points.points.append(Point32(x=x1, y=y1, z=0))
                points.points.append(Point32(x=x2, y=y2, z=0))
                points.points.append(Point32(x=x1, y=y2, z=0))
                points.points.append(Point32(x=x2, y=y1, z=0))
                self.point_pub.publish(points)
        else:
            self.point_pub.publish(Polygon())


        # Show object hull for visual check
        cv2.imshow("Image window", cv_image)
        cv2.imshow("Color Image window", color_mask)
        cv2.waitKey(3)

        # Convert OpenCV image back to ROS image message if you need to publish it
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

def main(args):
    ic = DetectBot()
    rospy.init_node('yellow_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

    
