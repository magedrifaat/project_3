#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Float64
from geometry_msgs.msg import Twist 

def listner():
    s = rospy.wait_for_message('cmd_vel', Twist)
    v = 10 * s.linear.x
    # v= 0
    w = 10 * s.angular.z
    return v,w

def getEffort(loc):
    topic_name = 'control_effort_r'
    if loc == 'r':
        topic_name = 'control_effort_r'
    elif loc == 'l':
        topic_name = 'control_effort_l'
    m = rospy.wait_for_message(topic_name, Float64)
    pwm = m.data
    return pwm


def getMotorsSpeed(v,w):
    ## v = (vr + vl) / 2 
    ## w = (vr - vl) / l
    vr = (2*v + l*w) / 2   #velocity of right wheel
    vl = 2*v - vr    #velocity of left wheel
    wr = vr / r # rotational speed of wheel in rad/sec
    wl = vl / r

    return wr, wl 


if __name__ == "__main__":
    rospy.init_node('pid_test', anonymous= False)
    pub_r = rospy.Publisher('target_r', Float64, queue_size = 10, latch= True)
    pub_pwm_r = rospy.Publisher('pwm_r', Float32, queue_size = 10)
    pub_l = rospy.Publisher('target_l', Float64, queue_size = 10)
    pub_pwm_l = rospy.Publisher('pwm_l', Float32, queue_size = 10)


    rate = rospy.Rate(10)
    l = 0.22 #length of space between the wheels
    r = 0.0325 # wheel red.
    v = 0 
    w = 0 
    wr = 0
    wl = 0
    # direction = "f"
    pwm_r = 0
    pwm_l = 0

    while not rospy.is_shutdown():
        v,w = listner()
        wr, wl = getMotorsSpeed(abs(v),w)
        # print(wr)
        pub_r.publish(wr)
        pub_l.publish(wl)
        # print('in')
       
        if v == 0 and w == 0:
            pwm_r = 0
            pwm_l = 0
        else:
            pwm_r += getEffort('r')
            pwm_l += getEffort('l')
            pwm_r= min(255, max(20, pwm_r))
            pwm_l= min(255, max(20, pwm_l))
        if v < 0:
          pub_pwm_r.publish(-pwm_r)
          pub_pwm_l.publish(-pwm_l)
        else:
          pub_pwm_r.publish(pwm_r)
          pub_pwm_l.publish(pwm_l)
