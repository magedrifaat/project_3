#!/usr/bin/env python

import rospy
import csv
from nav_msgs.msg import OccupancyGrid
import numpy as np 

def listner():
    map = rospy.wait_for_message('/map', OccupancyGrid)
    return map.data,map.info.width

def crop(d):
    m =[]
    for row in d :
        if row.count(-1) != len(row):
            m.append(row)
    m = np.array(m)
    width = np.shape(m)[1]
   
    for i in range(width,0,-1):
        if np.count_nonzero(m[:,i-1] == -1) == np.shape(m)[0]:
            m = np.delete(m,i-1,1)
    print(np.shape(m))
    return list(m)

if __name__ == "__main__":
    rospy.init_node('map_catcher',anonymous=True)
    data,width = listner()
    rows = [data[width*i:width*(i+1)] for i in range(width + 1)]
    map = crop(rows)

    with open('/home/mahmoud/catkin_ws/src/project_3/src/scripts/plannig/map.csv', 'w') as f:
        writer = csv.writer(f)
        for row in map:
            writer.writerow(row)
