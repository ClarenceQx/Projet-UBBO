#!/usr/bin/env python

import rospy
import numpy as np
from icp_algo import icp_algo
#import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan

class ICP:
    
    def __init__(self, coefs=[1,0,0], thresh=0.1):
        rospy.init_node('ICP_node')        
        
        ### initialize variables
        self.prev_map = None
        self.total_transo = np.zeros((1,3))
        self.coefs = coefs
        self.thresh = thresh
        
        #### subscribers/publishers
        rospy.Subscriber('/scan', LaserScan, self.icp_callback)

    def build_map(self, distances, delta, angle_min):
        theta = angle_min
        pts = []
        for d in distances:
            pts.append(self.polar2cart(theta, d))
            theta += delta
        return pts
            
    def polar2cart(self, theta, d):
        return (d*np.cos(theta), d*np.sin(theta))
    
    def icp_callback(self, data):
        angle_min = data.angle_min
        angle_incr = data.angle_increment
        new_scan = data.ranges
        new_map = self.build_map(new_scan, angle_incr, angle_min)
        """
        plt.scatter(*zip(*my_map), marker='x', color='red')
        plt.axis("off")
        plt.savefig('/home/clarence/Bureau/test.png')
        print('fig saved')
        """
        if self.prev_map is not None:
            transformation_history, _ = icp_algo(self.prev_map, new_map, max_iterations=100, distance_threshold=self.thresh, point_pairs_threshold=900, verbose=False)
            self.total_transo += np.where(np.abs(transformation_history) > 0.005, transformation_history, 0)*self.coefs
            print(self.total_transo)
        else:
            print("Pas de scan precedent. En attente d'un nouveau scan")
        self.prev_map = new_map

if __name__ == '__main__':
    """ main """
    icp = ICP([3, 0, 0], 0.25) # ICP for calculating travel distance
    #icp = ICP([0,0,1], 5) # ICP for calculating rotation (rad)
    rospy.spin()