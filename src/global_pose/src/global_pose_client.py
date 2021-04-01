#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyRequest

rospy.init_node('global_pose_client')
rospy.wait_for_service('get_global_pose')
service = rospy.ServiceProxy('/get_global_pose', Empty)
serv_object = EmptyRequest()
_ = service(serv_object)