#!/usr/bin/env python
import rospy
from tf import TransformBroadcaster, transformations
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion
import numpy as np

class odom:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.r = 0.065 / 2
        self.l = 0.23
        self.w_r = 0
        self.w_l = 0
        self.v = 0
        self.w = 0
        self.vx = 0
        self.vy = 0

    def listner(self):
        encoder_r = rospy.wait_for_message('/Encoder_r', Float64)
        encoder_l = rospy.wait_for_message('/Encoder_l', Float64)
        self.w_r = encoder_r.data
        self.w_l = encoder_l.data

    def compute(self, t):
        self.v = (self.w_r + self.w_l) / 2
        self.w = (self.w_r - self.w_l) * self.r / self.l


        self.vx = self.v * np.cos(self.theta)
        self.vy = self.v * np.sin(self.theta)
        self.theta += self.w * t

        self.x += self.vx * t 
        self.y += self.vy * t 

        # self.x += (vx * np.cos(self.theta) - vy * np.sin(self.theta)) * t
        # self.x += (vx * np.sin(self.theta) + vy * np.cos(self.theta)) * t


if __name__ == "__main__":
    rospy.init_node('odom',anonymous=False)
    dt = 0
    last_time = rospy.get_rostime().to_sec()
    i = 1
    odom_pub = rospy.Publisher('odom', Odometry, latch= True, queue_size=50)
    state = odom()
    odom_quatr = Quaternion()
    odom_msg = Odometry()
    rate = rospy.Rate(10)
    br = TransformBroadcaster()

    while not rospy.is_shutdown():
        curent_time = rospy.get_rostime().to_sec()
        dt = (curent_time - last_time)
        if dt < 0:
            dt = 0
        # last_time = rospy.get_rostime().to_sec()
        last_time = curent_time
        
        state.listner()
        state.compute(dt)

        odom_quatr = transformations.quaternion_from_euler(0, 0 , state.theta)

        br.sendTransform((state.x, state.y, 0),
                            transformations.quaternion_from_euler(0, 0 , state.theta),
                            rospy.get_rostime(),
                            'base_link',
                            'odom')


        odom_msg.header.stamp = rospy.get_rostime()
        odom_msg.header.frame_id = 'odom'
        odom_msg.pose.pose.position.x = state.x
        odom_msg.pose.pose.position.y = state.y
        odom_msg.pose.pose.position.z = 0
        odom_msg.pose.pose.orientation.x = odom_quatr[0]
        odom_msg.pose.pose.orientation.y = odom_quatr[1]
        odom_msg.pose.pose.orientation.z = odom_quatr[2]
        odom_msg.pose.pose.orientation.w = odom_quatr[3]
        odom_msg.child_frame_id = 'base_link'
        odom_msg.twist.twist.linear.x = state.vx
        odom_msg.twist.twist.linear.y = state.vy
        odom_msg.twist.twist.angular.z = state.theta


        odom_pub.publish(odom_msg)
        rate.sleep()
   