#! /usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import convolve2d
from std_srvs.srv import Empty, EmptyResponse
from global_pose.msg import global_pose
import roslaunch

def get_map():
    node = roslaunch.core.Node('map_server', 'map_saver', args='f ~/Bureau/maps/one_map')
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    gmapping_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/clarence/Bureau/catkin_ws/src/my_mapping_launcher/launch/my_gmapping_launch_2.launch"])
    gmapping_launch.start()
    
    rospy.loginfo("service started")
    rospy.sleep(1)
    
    process = launch.launch(node)
    rospy.sleep(1)
    process.stop()
    
    gmapping_launch.shutdown()

def callback_service(request):
    print("Global pose service called")
    # get map for LaserScan
    #get_map()
    # upload global map
    gmap = plt.imread("/home/clarence/Bureau/maps/grande_map.pgm")
    gmap = gmap[np.mean(gmap, axis=1) != 205]
    gmap = gmap.T
    gmap = gmap[np.mean(gmap, axis=1) != 205]
    gmap = gmap.T
    # pattern matching
    img = plt.imread("/home/clarence/Bureau/maps/map.pgm") #change in par from LaserScan
    pattern = img[np.mean(img, axis=1) != 205]
    pattern = pattern.T
    pattern = pattern[np.mean(pattern, axis=1) != 205]
    pattern = pattern.T
    pattern = pattern - np.mean(pattern)
    
    convo = convolve2d(in1=gmap, in2=pattern[::-1, ::-1], mode='same')
    # get global position
    m = 0
    pos = (0,0)
    for i in range(convo.shape[0]):
        for j in range(convo.shape[1]):
            if convo[i,j] > m:
                pos = (i,j)
                m = convo[i,j]
            
    pose.x = pos[0]
    pose.y = pos[1]
    pose.orientation = 0
    pub.publish(pose)
    return EmptyResponse()

rospy.init_node('global_pose_server')
service = rospy.Service('get_global_pose', Empty, callback_service)
pub = rospy.Publisher('/pos_lidar', global_pose)
pose = global_pose()
rospy.spin()