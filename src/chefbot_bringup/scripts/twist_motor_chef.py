#!/usr/bin/env python
"""
   twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack
   
"""


import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from math import pi 

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
        rospy.Subscriber('teleop_chef_node/cmd_vel', Twist, self.twistCallback)
    
    
        self.rate = rospy.get_param("~rate", 50) #50
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2) #2
        self.left = 0
        self.right = 0
        self.dx_actu=0
        self.dx_past=0
        self.dy_actu=0
        self.dy_past=0
        self.dr_actu=0
        self.dr_past=0
        self.dx=0
        self.dy=0
        self.dr=0
        self.rayon=0.05
        self.base_width=0.215
        
    #############################################################
    def spin(self):
    #############################################################
    
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)

        self.ticks_since_target = self.timeout_ticks
        """
        self.left = 0
        self.right = 0
        self.pub_lmotor.publish(self.left)
        self.pub_rmotor.publish(self.right)
        rospy.sleep(10.)
        """
        rospy.logwarn("ready to move")
        
        ###### main loop  ######
        while not rospy.is_shutdown():
            #self.right=1
            #self.left=1
            #self.right = self.dx + self.dr
            #self.left = self.dx - self.dr
            #self.pub_lmotor.publish(self.left)
            #self.pub_rmotor.publish(self.right)
            #rospy.loginfo("target right %f left %f", self.right, self.left)
            #idle.sleep()
        
            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()
            
        self.left = 0
        self.right = 0
        self.pub_lmotor.publish(self.left)
        self.pub_rmotor.publish(self.right)
        rospy.sleep(2.)
                
    #############################################################
    def spinOnce(self):
    #############################################################
    
        # dx = (l + r) / 2
        # dr = (r - l) / w
            
        #self.right = 1.0 * self.dx + self.dr * self.w / 2 
        #self.left = 1.0 * self.dx - self.dr * self.w / 2
        self.right = (self.dx + self.dr)/(2*pi*self.rayon)
        self.left = (self.dx - self.dr)/(2*pi*self.rayon)
        #self.right=1
        #self.left=1
        #rospy.loginfo("publishing: (%d, %d)", self.left, self.right) 
                
        self.pub_lmotor.publish(self.left)
        self.pub_rmotor.publish(self.right)
            
        self.ticks_since_target += 1

    #############################################################
    def twistCallback(self,msg):
    #############################################################
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        
        self.dx_past=self.dx_actu
        self.dy_past=self.dy_actu
        self.dr_past=self.dr_actu
        
        self.dx_actu = msg.linear.x
        self.dr_actu = (msg.angular.z)*self.base_width/2
        self.dy_actu = msg.linear.y
        
        if self.dr_actu!=self.dr_past:
            self.dr=self.dr_actu
        
        if self.dx_actu!=self.dx_past:
            self.dx=self.dx_actu
            
        if self.dy_actu!=self.dy_past:
            self.dy=self.dy_actu
            
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    twistToMotors = TwistToMotors()
    twistToMotors.spin()
