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
    
        
        self.pub_lmotor = rospy.Publisher('motor_cmd_l',Float32,queue_size=10) 
        self.pub_rmotor = rospy.Publisher('motor_cmd_r',Float32,queue_size=10) 
        rospy.Subscriber('teleop_chef_node/cmd_vel', Twist, self.twistCallback)
    
    
        self.rate = rospy.get_param("~rate", 50) #50
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2) #2
        self.left = 0
        self.right = 0
     
        self.dx=0
        self.dy=0
        self.dr=0
        self.rayon=0.05
        self.base_width=0.215
        self.RPS2PWM = 255/1.5; # max = 1.7 tours/sec
        self.VL2PWM=self.RPS2PWM*(2*pi*self.rayon)
        
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
        self.right = (self.dx + self.dr)*self.RPS2PWM/(2*pi*self.rayon)
        self.left = (self.dx - self.dr)*self.RPS2PWM/(2*pi*self.rayon)
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
        
        self.dx = msg.linear.x
        self.dr = (msg.angular.z)*self.base_width/2
        self.dy = msg.linear.y
        
            
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    twistToMotors = TwistToMotors()
    twistToMotors.spin()
