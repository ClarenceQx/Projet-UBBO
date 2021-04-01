#!/usr/bin/env python
import rospy
import numpy as np
from icp_algo import icp_algo
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

prev_map = None
next_map = True

def cont(msg):
    global next_map
    next_map = True

def build_map(distances, delta, angle_min):
    theta = angle_min
    pts = []
    for d in distances:
        pts.append(polar2cart(theta, d))
        theta += delta
    return pts
        
def polar2cart(theta, d):
    return (d*np.cos(theta), d*np.sin(theta))

def icp_callback(data):
    global prev_map
    global next_map
    if next_map:
        angle_min = data.angle_min
        angle_incr = data.angle_increment
        new_scan = data.ranges
        new_map = build_map(new_scan, angle_incr, angle_min)
        
        plt.scatter(*zip(*new_map), marker='x', color='red')
        plt.axis("off")
        plt.show()
        #plt.savefig('/home/clarence/Bureau/test.png')
        #print('fig saved')
        
        if prev_map is not None:
            transformation_history, _ = icp_algo(prev_map, new_map, max_iterations=500, distance_threshold=0.1, point_pairs_threshold=700, verbose=True)
            print(transformation_history)
        else:
            print("Pas de scan precedent. En attente d'un nouveau scan")
        prev_map = new_map
        next_map = False


rospy.init_node('test')
rospy.Subscriber('/scan', LaserScan, icp_callback)
rospy.Subscriber('/continue', Int32, cont)
rospy.spin()