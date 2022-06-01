#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

def talker():
  global v 
  pub_par = rospy.Publisher("/command", Float32 ,queue_size=10 ,latch=True)
  rospy.init_node("command", anonymous= True)
  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    print("w:forward , s:stop: ")
    command = input()
    if command == "w":
      v = v + 0.02
    elif command == "s":
      v = 0
    print(v)
    
    pub_par.publish(v)
    rate.sleep()

if __name__ == "__main__":
  v = 0
  try:
    talker()
  except rospy.ROSInterruptException:
      pass

