#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from cv_bridge import CvBridge

from lane_value_publisher import get_lane_value, set_publisher

left_lane_value = -1
right_lane_value = 1
exit_lane_value = None
FPS = None
new_data = False

def callback(ros_rgb_image):
    global left_lane_value
    global right_lane_value
    global exit_lane_value
    global FPS
    global new_data
    # if rospy.get_rostime() - ros_rgb_image.header.stamp > rospy.Duration(0.05):
    #     return
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(ros_rgb_image)
    values = get_lane_value(cv_image)
    if values is not None:
        left_lane_value = values["left_lane"]
        right_lane_value = values["right_lane"]
        exit_lane_value = values["exit_lane"]
        FPS = values["FPS"]
        new_data = True

def listener():
    rospy.init_node('image_lane_finder', anonymous=True)

    rospy.Subscriber("/camera/rgb/image_raw", Image, callback, queue_size=1)

def publisher():
    global new_data
    
    left_pub = rospy.Publisher('left_lane_value', Float32, queue_size=10)
    right_pub = rospy.Publisher('right_lane_value', Float32, queue_size=10)
    exit_pub = rospy.Publisher('exit_lane_value', Float32, queue_size=10)
    fps_pub = rospy.Publisher('FPS', Float32, queue_size=10)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        new_data = False
        left_pub.publish(left_lane_value)
        right_pub.publish(right_lane_value)
        exit_pub.publish(exit_lane_value if exit_lane_value is not None else -2)
        fps_pub.publish(FPS if FPS is not None else 0)

        rate.sleep()
        while not new_data:
            rate.sleep()

if __name__ == '__main__':
    listener()
    set_publisher(rospy.Publisher('image_cv2', Image, queue_size=1))
    publisher()