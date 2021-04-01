#! /usr/bin/env python

import roslaunch
import matplotlib.pyplot as plt
from scipy.signal import convolve2d
import numpy as np
import rospy
from std_msgs.msg import Int32
#from global_pose.msg import global_pose

def show_img(img, title="",save=None):
    plt.figure(figsize=(20, 10))
    plt.imshow(img, cmap='gray')
    plt.title(title)
    plt.colorbar()
    if save != None:
        plt.savefig(save)
        print("Fig saved")
    plt.show()

class GlobalPose:
    
    def __init__(self, globalMapLoc, localMapLoc, gmappingLaunchLoc):
        rospy.init_node('globalPose_node')        
        
        ### initialize variables
        self.globalMapLoc = globalMapLoc
        self.localMapLoc = localMapLoc
        self.gmappingLaunchLoc = gmappingLaunchLoc
        self.process = None
        self.gmappingLaunch = None
        
        #### subscribers/publishers 
        #self.pub_pos = rospy.Publisher('/pos_lidar', global_pose)
        self.pub_pos = rospy.Publisher('/pos_lidar', Int32)
        
    def _launchMapSaver(self):
        node = roslaunch.core.Node('map_server', 'map_saver', args='-f '+ self.localMapLoc[:-4])
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        self.process = launch.launch(node)
    
    def _launchGmapping(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.gmapping_launch = roslaunch.parent.ROSLaunchParent(uuid, [self.gmappingLaunchLoc])
        self.gmapping_launch.start()
    
    def _stopMapSaver(self):
        if self.process is not None:
            self.process.stop()
    
    def _stopGmapping(self):
        if self.gmappingLaunch is not None:
            self.gmappingLaunch.shutdown()
    
    def getMap(self):
        self._launchGmapping()
        
        rospy.loginfo("Getting map")
        rospy.sleep(1)
        
        self._launchMapSaver()
        
        rospy.sleep(1)
        
        self._stopGmapping()
        self._stopMapSaver()
        
    def _greyRemoving(self, img):
        img = img[np.mean(img, axis=1) != 205]
        img = img.T
        img = img[np.mean(img, axis=1) != 205]
        img = img.T
        return img - np.mean(img)
    
    def spinOnce(self):
        self.getMap()
        
        globalMap = plt.imread(self.globalMapLoc)
        globalMap_resized = self._greyRemoving(globalMap)
        
        
        localMap = plt.imread(self.localMapLoc)
        localMap_resized = self._greyRemoving(localMap)
        
        convo = convolve2d(in1=globalMap_resized, in2=localMap_resized[::-1, ::-1], mode='same')
        
        show_img(convo, save="/home/clarence/Bureau/Presentation_UBBO/Pattern_matching/png/convo.png")        
        
        m = 0
        pos = (0,0)
        for i in range(convo.shape[0]):
            for j in range(convo.shape[1]):
                if convo[i,j] > m:
                    pos = (i,j)
                    m = convo[i,j]
        """
        pose = global_pose()
        pose.x = pos[0]
        pose.y = pos[1]
        pose.orientation = 0
        """
        print(pos)
        self.pub_pos.publish(1)

if __name__ == '__main__':
    """ main """
    globalPose = GlobalPose("/home/clarence/Bureau/Presentation_UBBO/Pattern_matching/map.pgm", "/home/clarence/Bureau/Presentation_UBBO/Pattern_matching/one_map.pgm", "/home/clarence/Bureau/catkin_ws/src/my_mapping_launcher/launch/my_gmapping_launch_2.launch")
    globalPose.spinOnce()