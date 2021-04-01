#! /usr/bin/env python

import roslaunch
import matplotlib.pyplot as plt
import numpy as np
import rospy

rospy.init_node('test')

node = roslaunch.core.Node('map_server', 'map_saver', args='-f /home/clarence/Bureau/Presentation_UBBO/test')
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

# pattern matching
img = plt.imread("/home/clarence/Bureau/Presentation_UBBO/test.pgm")
print('img uploaded')