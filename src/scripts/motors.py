#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from pid_controller import PID
from geometry_msgs.msg import Twist 

def listner():
    s = rospy.wait_for_message('cmd_vel', Twist)
    v = 10 * s.linear.x
    # v= 0
    w = 10 * s.angular.z
    return v,w

def getMotorsSpeed(v,w):
    ## v = (vr + vl) / 2 
    ## w = (vr - vl) / l
    vr = (2*v + l*w) / 2   #velocity of right wheel
    vl = 2*v - vr    #velocity of left wheel
    wr = vr / r # rotational speed of wheel in rad/sec
    wl = vl / r

    return wr, wl 



if __name__ == "__main__":
    rospy.init_node('motors_pid', anonymous= False)
    pub_r = rospy.Publisher('pwm_r', Float32, queue_size = 10)
    pub_l = rospy.Publisher('pwm_l', Float32, queue_size = 10)

    rate = rospy.Rate(10)
    l = 0.2 #length of space between the wheels
    r = 0.1 # wheel red.
    v = 0 
    w = 0 
    wr = 0
    wl = 0
    direction = "f"
    pwm_r = 0
    pwm_l = 0 
    pid_r = PID(kp = 1.3, kd=0, ki= 0.6, topic='Encoder_r')
    pid_l = PID(kp = 2.4, kd=0, ki= 0.8, topic='Encoder_l')

    while not rospy.is_shutdown():
        v,w = listner()
        if v>0:
            direction = "f"
        elif v<0:
            direction ='l'
            v = -v
        print(v)
        wr, wl = getMotorsSpeed(v,w)

        pid_r.ref = wr
        pid_r.error_listner()
        pid_r.compute()
        pwm_r += pid_r.output

        pid_l.ref = wl
        pid_l.error_listner()
        pid_l.compute()
        pwm_l += pid_l.output

        print("wr: " , wr , " wl: " , wl)
        if v != 0:
            pwm_r= min(255, max(20, pwm_r))
            pwm_l= min(255, max(20, pwm_l))
            # pwm_r= 40
            # pwm_l= 0
                
        elif v == 0:
            pwm_r = 0
            pwm_l = 0

        if direction == 'f':
            pub_r.publish(pwm_r)
            pub_l.publish(pwm_l)
     

        elif direction == 'l':
            pub_r.publish(-pwm_r)
            pub_l.publish(-pwm_l)


        rate.sleep()

