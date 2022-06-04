#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32,Float64
from low_pass_filter import filter


class PID:
    def __init__(self, kp = 1.1, kd = 0.6 , ki = 0.01, rate = 10 ,topic = '/right_lane_value'):
        self.kp = kp
        self.ki = ki 
        self.kd = kd 
        self.past_error = 0
        self.error_sum = 0
        self.e = 0
        self.output  = 0 
        self.t = 1 / rate
        self.topic = topic
        self.ref = 0 
        self.v_prev = 0
        self.vFilt = 0
        self.a = [0]
        self.b = [1,0]

    def lowPassfilter(self):
        f = filter(35)
        self.a , self.b = f.filterCofficient()

    def error_listner(self):
        value = rospy.wait_for_message(self.topic, Float32)
        # self.lowPassfilter()
        # self.vFilt = self.a[0] * self.vFilt + self.b[0]* value.data + self.b[1] * self.v_prev   #lowPass filter equation
        # self.v_prev = value.data
        # self.e = self.ref - self.vFilt
        self.e = self.ref - value.data
    
    def compute(self):
        if self.e != 0 :
            p  = self.e * self.kp 
            d = self.kd * (self.e - self.past_error) / self.t
            i = 0
            i = (i + self.e * self.t)
            self.output = p + self.ki * i + d 
            self.past_error =  self.e 
            # self.error_sum += self.e 

        # else:
        #     self.past_error =  0 
        #     self.error_sum = 0
        #     self.output = 0