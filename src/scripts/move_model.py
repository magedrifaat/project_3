#!/usr/bin/env python
from decimal import MAX_EMAX
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from pid_controller import PID
import numpy as np 

class movement :

    def __init__(self):
        rospy.init_node('move_robot_node', anonymous=False)
        self.pub_move = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.move = Twist()
        self.f = 10 
        self.t = 0.5 
        self.v_max = 0
        self.v = 0
        self.v_step = self.v_max / (self.f * self.t)
        self.u = 0 
        self.min_dist = 1 #min distance before avoide in meters 
        self.lane_width = 0.23 #in meters
        self.curent_dis = -1
        self.topic_names = ['/left_lane_value', '/right_lane_value']
        self.topic = self.topic_names[1]
        self.changeing = 'done'
        self.beside = False



    def lidarListner(self, min_angle, max_angle):
        dis = []
        lidar = rospy.wait_for_message('/scan', LaserScan)
        data =lidar.ranges 
        avr = -1
        for i in range(min_angle, max_angle):
            reading = data[i-1]
            if str(reading) != "inf":
                dis.append(reading)        
        if len(dis) != 0:
            avr = sum(dis) / len(dis)
        return avr

    def publish_vel(self):
        self.pub_move.publish(self.move)

    def move_forward(self):
        if self.v <= self.v_max:
            self.move.linear.x= self.v
            self.v += self.v_step
        self.move.linear.x = self.v_max
        self.move.angular.z = 0.5* self.u 
        # print('on')


    def move_backward(self):    
        if self.v != -self.v_max:
            self.move.linear.x= self.v
            self.v -= self.v_step  
        self.move.angular.z=0.0

    def stop(self):        
        self.v  = 0
        self.move.linear.x= self.v
        self.move.angular.z=0.0  

    def change(self):
        self.changeing = 'running'
        i = self.topic_names.index(self.topic)
        self.topic =self.topic_names[(i + 1) % len(self.topic_names)]

    def aviod(self):
        max_angle = round(np.arctan2(self.lane_width,self.min_dist) * (180 / np.pi))
        min_angle = round(np.arctan2(-self.lane_width, self.min_dist) * (180/ np.pi))
        dis = self.lidarListner(min_angle, max_angle)
        if dis != -1:
            self.curent_dis = dis

    def end_change(self):
        min_angle = 0
        max_angle = 0 
        if self.topic == self.topic_names[0]:
            min_angle = 90
            max_angle = 120
        elif self.topic == self.topic_names[1]:
            min_angle = 240
            max_angle = 270
        
        beside_distance = self.lidarListner(min_angle, max_angle)

        if beside_distance != -1:
            if beside_distance < 0.2 :
                self.beside = True
            else:
                self.beside = False
        
        




if __name__ == "__main__":
    mov = movement()
    status = 'stop'
    pid = PID(kp = 1.1, kd = 0.6 , ki = 0.01, rate = mov.f)
    rate = rospy.Rate(mov.f)
    while not rospy.is_shutdown() :
        pid.topic = mov.topic
        pid.error_listner()
        # if mov.changeing != 'done' and abs(pid.e) < 0.05:
        #     mov.changeing = 'done'

        print(mov.changeing)
        c = rospy.wait_for_message("/command", Float32)
        if c.data == 0:
            status = "stop"
        else:
            status = "forward"
            mov.v_max = c.data 
        
        print("lane erroe:", pid.e)

        if mov.changeing == 'done':
            print('cheeking obstacle')
            mov.aviod()
            if mov.curent_dis < 1 and mov.curent_dis != -1:
                print(pid.topic)
                status = 'lane_change'
        
        # elif mov.changeing == 'running':
        #     mov.end_change()
        #     # if mov.beside:
        #     print('********')
        #     print('there is something beside')
        #     print('********')

        pid.compute()
        mov.u = pid.output
        print('status: ', status)
        if status == 'forward':
            mov.move_forward()

        elif status == 'backward':
            mov.move_backward()

        elif status == 'stop':
            mov.stop()
        elif status == 'lane_change':
            print('change')
            mov.change()

        mov.publish_vel()
        print("############")
        rate.sleep()

#rosservice call /gazebo/reset_simulation "{}"