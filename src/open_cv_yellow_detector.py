#!/usr/bin/env python3
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

from utils import get_limits


class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic", ImageMsg, queue_size=10)
        self.point_pub = rospy.Publisher("point_topic", Polygon, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image", ImageMsg, self.callback)

    def callback(self, data):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        (rows,cols,channels) = cv_image.shape
        print("Number of rows", rows)
        print("Number of columns", cols)

        # Yellow color detection
        yellow = [0, 255, 255]  # yellow in BGR colorspace

        hsvImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lowerLimit, upperLimit = get_limits(color=yellow)
        mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
 

       
        mask_ = Img.fromarray(mask)
 
        bbox = mask_.getbbox()
        #print("Bbox", bbox)

        if bbox is not None:
            x1, y1, x2, y2 = bbox
            cv_image = cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 5)

            points = Polygon()
            points.points.append(Point32(x=x1, y=y1, z=0))
            points.points.append(Point32(x=x2, y=y2, z=0))
            points.points.append(Point32(x=x1, y=y2, z=0))
            points.points.append(Point32(x=x2, y=y1, z=0))

            self.point_pub.publish(points)

            print(f"Published points: [({x1}, {y1}), ({x2}, {y2}), ({x1}, {y2}), ({x2}, {y1})]")
            print("Area", (x2-x1)*(y2-y1))

        #contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        
        cv2.imshow("Image window", cv_image)

        cv2.waitKey(3)

        # Convert OpenCV image back to ROS image message if you need to publish it
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

    
