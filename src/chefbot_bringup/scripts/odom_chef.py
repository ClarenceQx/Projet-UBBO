#!/usr/bin/env python

"""
   diff_tf.py - follows the output of a wheel encoder and
   creates tf and odometry messages.
"""

import rospy
#import roslib
#roslib.load_manifest('differential_drive')
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
#from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import  Int32

import numpy as np
from tf.broadcaster import TransformBroadcaster

#from tf.transformations import quaternion_from_euler


#############################################################################
class DiffTf:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("odom_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        cpr=16
        gear_ratio=90
        rayon=0.05
        ticks_meter_data=cpr*gear_ratio/(2*pi*rayon)
        base_width_data=0.215


        
        #### parameters #######
        self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the transform
        self.ticks_meter = float(rospy.get_param('ticks_meter', ticks_meter_data))  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(rospy.get_param('~base_width', base_width_data)) # The wheel base width in meters
        
        self.base_frame_id = rospy.get_param('~base_frame_id','base_footprint') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
        
        self.encoder_min = rospy.get_param('encoder_min', -2147483648)
        self.encoder_max = rospy.get_param('encoder_max', 2147483648)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
 
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        
        # internal data
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.tour_init_left= True
        self.tour_init_right= True
        self.encod_left_init=0
        self.encod_right_init=0
        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0                  # position in xy plane 
        self.y = 0
        self.th = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.dy = 0
        self.theta=0
        self.dxt=0

        self.then = rospy.Time.now()
        
        
        # subscriptions
        rospy.Subscriber("lwheel", Int32, self.lwheelCallback)
        rospy.Subscriber("rwheel", Int32, self.rwheelCallback)
        self.odomPub = rospy.Publisher("odom", Odometry,queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()
        
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
       
     
    #############################################################################
    def update(self):
    #############################################################################
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()
            

            # calculate odometry
            if self.enc_left == None:
                d_left = 0
                d_right = 0
            else:
                d_left = (self.left - self.enc_left) / self.ticks_meter
                d_right = (self.right - self.enc_right) / self.ticks_meter
            self.enc_left = self.left
            self.enc_right = self.right
            
            #rospy.logwarn("%f %f",d_left,d_right)
            # distance traveled is the average of the two wheels 
            d = ( d_left + d_right ) / 2
            # this approximation works (in radians) for small angles
            th = np.arctan(( d_right - d_left ) / self.base_width)
            # calculate velocities
            self.dxt = d / elapsed
            self.dr = th / elapsed
            
            """
            dencod_r=self.right-self.enc_right
            dencod_l=self.left-self.enc_left
            
            if (dencod_l<=dencod_r):
                ds=(d_left+d_right)/2
                dtheta=(d_right-d_left)/(2*self.base_width)
            
                self.dx=ds*np.cos(self.theta+dtheta/2)
                self.dy=ds*np.sin(self.theta+dtheta/2)
            
            if (dencod_l>dencod_r):
                ds=-(d_left+d_right)/2
                dtheta=(d_right-d_left)/(2*self.base_width)
            
                self.dx=-ds*np.cos(self.theta+dtheta/2)
                self.dy=-ds*np.sin(self.theta+dtheta/2)
            
            #rospy.loginfo("%f %f %f %f %f",self.dx,self.dy,ds,self.theta,dtheta)
            if (d!=0):                
                self.x+=self.dx
                self.y+=self.dy
                #rospy.loginfo("%f %f",self.x, self.y)
            if (th!=0):
                self.theta+=dtheta
                
            #rospy.loginfo("%f %f %f",self.x, self.y, self.theta)
            if (self.theta > np.pi):
                self.theta=-np.pi
            if (self.theta <-np.pi):
                self.theta=np.pi
            """
            
            if (d != 0):
                # calculate distance traveled in x and y
                xtemp = cos( th ) * d
                ytemp = -sin( th ) * d
                # calculate the final position of the robot
                self.x = self.x + ( cos( self.th ) * xtemp - sin( self.th ) * ytemp )
                self.y = self.y + ( sin( self.th ) * xtemp + cos( self.th ) * ytemp )
            if( th != 0):
                self.th = self.th + th
                      
                
            # publish the odom information
            quaternion = Quaternion()
            quaternion.x =0.0 #self.theta*360/2/np.pi 
            quaternion.y = 0.0
            quaternion.z = sin( self.th / 2 )
            quaternion.w = cos( self.th / 2 )
            
            
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
                )
            
            
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.pose.covariance=[0.1,0.0,0.0,0.0,0.0,0.0,0.0,0.1,0.0,0.0,0.0,0.0,0.0,0.0,1000000.0,0.0,0.0,0.0,0.0,0.0,0.0,1000000.0,0.0,0.0,0.0,0.0,0.0,0.0,1000000.0,0.0,0.0,0.0,0.0,0.0,0.0,0.05]	
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.dr
            odom.twist.covariance=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
            self.odomPub.publish(odom)
            
            


    #############################################################################
    def lwheelCallback(self, msg):
    #############################################################################
        if (self.tour_init_left==True):
            self.encod_left_init=msg.data
            self.tour_init_left=False
                    
        enc = msg.data-self.encod_left_init
        
        if (enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
            self.lmult = self.lmult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
            self.lmult = self.lmult - 1
            
        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min)) 
        
        self.prev_lencoder = enc
        
    #############################################################################
    def rwheelCallback(self, msg):
    #############################################################################
        if (self.tour_init_right==True):
            self.encod_right_init=msg.data
            self.tour_init_right=False        
        
        enc = msg.data-self.encod_right_init
        
        if(enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
            self.rmult = self.rmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap):
            self.rmult = self.rmult - 1
            
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))

        self.prev_rencoder = enc

#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    diffTf = DiffTf()
    diffTf.spin()
    
    
   
