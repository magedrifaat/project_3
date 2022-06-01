#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

if __name__ == '__main__':
    rospy.init_node('camera_publisher', anonymous=True)
    pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=1)
    cap = cv2.VideoCapture(0)
    rate = rospy.Rate(30)
    bridge = CvBridge()
    while not rospy.is_shutdown():
        _, image = cap.read()
        ros_image = bridge.cv2_to_imgmsg(image, encoding="passthrough")
        pub.publish(ros_image)
        rate.sleep()