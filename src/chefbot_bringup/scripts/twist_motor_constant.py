#!/usr/bin/env python
"""
   twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack
   
"""


import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist 

#############################################################
#############################################################
class TwistToMotors():
#############################################################
#############################################################

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("twist_to_motors")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
    
        self.w = rospy.get_param("~base_width", 0.21)
    
        self.pub_lmotor = rospy.Publisher('lwheel_vtarget', Float32,queue_size=10)
        self.pub_rmotor = rospy.Publisher('rwheel_vtarget', Float32,queue_size=10)
 
        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.left = 0
        self.right = 0
        
        self.count=0 

        self.dt=0
        
    #############################################################
    def spin(self):
    #############################################################
        
        idle = rospy.Rate(10)
        
        self.left = 0
        self.right = 0
        self.pub_lmotor.publish(self.left)
        self.pub_rmotor.publish(self.right)
        rospy.sleep(10.)
        
        self.tps_rec=rospy.Time.now()
        self.tps_init=self.tps_rec.to_sec()
        
        self.left = 1.6
        self.right = 1.6
        
        while (self.dt)<10:
            
            self.pub_lmotor.publish(self.left)
            self.pub_rmotor.publish(self.right)
            
            self.tps=rospy.Time.now()
            self.tps_act=self.tps.to_sec()
            self.dt=self.tps_act - self.tps_init
            
            rospy.loginfo("temps %0.2f %f %f", self.dt, self.right, self.left)
            idle.sleep()


        self.left = 0
        self.right = 0
        self.pub_lmotor.publish(self.left)
        self.pub_rmotor.publish(self.right)
        rospy.sleep(2.)
        rospy.loginfo("temps %0.2f %f %f", self.dt, self.right, self.left)
  

    
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    twistToMotors = TwistToMotors()
    twistToMotors.spin()
