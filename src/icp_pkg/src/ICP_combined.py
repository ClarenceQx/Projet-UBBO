#!/usr/bin/env python

import rospy
import numpy as np
from ICP import ICP
#import matplotlib.pyplot as plt
from std_msgs.msg import Float32
from geometry_msgs import Point

class ICP_combined:
    
    def __init__(self, icp_dist, icp_rot):
        rospy.init_node('icp_comb')        
        
        ### initialize variables
        self.icp_dist = icp_dist
        self.icp_rot = icp_rot
        self.position = [0, 0, 0]
        
        #### subscribers/publishers
        rospy.Subscriber('/'+icp_dist.name, Float32, self.dist_callback)
        rospy.Subscriber('/'+icp_rot.name, Float32, self.rot_callback)
        self.dist_publisher = rospy.Publisher('/pos_lidar', Point, queue_size=0)
        self.rot_publisher = rospy.Publisher('/rot_lidar', Float32, queue_size=0)
    
    def dist_callback(self, data):
        print('yes')
        self.position[0] += data*np.cos(self.position[2])
        self.position[1] += data*np.sin(self.position[2])
        point = Point(self.position[0], self.position[1], 0)
        rospy.loginfo("Distance :")
        self.dist_publisher.publish(point)
    
    def rot_callback(self, data):
        print('no')
        self.position[2] += data
        rospy.loginfo("Rotation :")
        self.rot_publisher.publish(self.position[2])

if __name__ == '__main__':
    """ main """
    dist_icp = ICP('icp_dist') # ICP for calculating travel distance
    rot_icp = ICP('icp_rot', [0,0,1], 5) # ICP for calculating rotation (rad)
    icp_comb = ICP_combined(dist_icp, rot_icp)
    rospy.spin()