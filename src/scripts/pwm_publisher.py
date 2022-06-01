#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

pwm = 0
def callback(vel_msg):
    global pwm
    pwm = min(255, max(0, vel_msg.linear.x * 255))

def talker():
    global pwm
    pub = rospy.Publisher('pwm', Float32 , queue_size=10)
    rospy.init_node('pwm_publisher', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback, queue_size=1)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(pwm)
        
        rate.sleep()



if __name__ == '__main__': 
    talker()
