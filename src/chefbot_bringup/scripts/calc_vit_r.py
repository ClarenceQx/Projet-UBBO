#!/usr/bin/env python

import rospy
import roslib
from math import pi
from std_msgs.msg import Int32,Int64
from std_msgs.msg import Float32
from numpy import array

    
######################################################
######################################################
class PidVelocity():
######################################################
######################################################


    #####################################################
    def __init__(self):
    #####################################################
        rospy.init_node("pid_velocity_r")
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)
        
        ### initialize variables
        
        self.vel = 0
        self.wheel_prev = 0
        self.wheel_latest = 0
        self.then = rospy.Time.now()
        self.wheel_mult = 0
        self.prev_encoder = 0
        self.t_latest_duration=0
        self.t_latest=0
        self.t_prec=0
        self.vit=0
        self.v_encod_max_norm=0.026 #0.148
        self.time_actu=0
        self.time_past=0
        
        self.delay_arduino=0.010
        
        self.cpr=16
        self.gear_ratio=90
        rayon=0.05
        self.ticks_meter_data=self.cpr*self.gear_ratio/(2*pi*rayon)
        self.RPS2PWM = 255/1.7; # max = 1.7 tours/sec
        

        self.out_min = rospy.get_param('~out_min',-255)
        self.out_max = rospy.get_param('~out_max',255)
        self.rate = rospy.get_param('~rate',10)
        self.rolling_pts = rospy.get_param('~rolling_pts',6)
        self.timeout_ticks = rospy.get_param('~timeout_ticks',4)
        self.ticks_per_meter = rospy.get_param('ticks_meter', self.ticks_meter_data)
        self.vel_threshold = rospy.get_param('~vel_threshold', 0.001)
        self.encoder_min = rospy.get_param('encoder_min', -2147483648)
        self.encoder_max = rospy.get_param('encoder_max', 2147483648)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
        self.prev_vel = [0.0] * self.rolling_pts
        self.wheel_latest = 0.0
        self.prev_pid_time = rospy.Time.now()

        #### subscribers/publishers 
        rospy.Subscriber("rwheel", Int32, self.wheelCallback) 
        rospy.Subscriber("time_encod_2", Int32, self.encodCallback)
        self.pub_vel = rospy.Publisher('wheel_vel_r', Float32,queue_size=10)
        self.pub_error = rospy.Publisher('error_vel_r',Float32,queue_size=10)
        
        
    #####################################################
    def spin(self):
    #####################################################
        self.r = rospy.Rate(self.rate) 
        self.then = rospy.Time.now()
        self.wheel_prev = self.wheel_latest
        self.t_latest_duration=rospy.Time.now()
        self.t_latest=self.t_latest_duration.to_sec()
        
        #rospy.loginfo("target %f", self.target)
        
        while not rospy.is_shutdown():
            self.spinOnce()
            self.r.sleep()
            
    #####################################################
    def spinOnce(self):
    #####################################################
            

        self.prev_vel = [0.0] * self.rolling_pts
 
        self.vel = 0.0
        self.vit=0.0
        
    
        self.getVitesse()
     
        self.r.sleep()
            

            
                
                
    #####################################################
    def getVitesse(self):
    #####################################################
        self.dt_v=self.time_actu-self.time_past
        #self.vit=(self.wheel_latest-self.wheel_prev)/self.dt_v
        
        if self.dt_v>0:            
            cur_vit=(self.wheel_latest-self.wheel_prev)*1000/(2*self.dt_v*self.cpr*self.gear_ratio)
        else:
            cur_vit=0
            #rospy.logwarn("dt negatif ou nul")
            
        self.appendVel(cur_vit)
        self.calcRollingVel()
        self.pub_vel.publish(self.vit)
        
            
    #####################################################
    def appendVel(self, val):
    #####################################################
        self.prev_vel.append(val)
        del self.prev_vel[0]
        
    #####################################################
    def calcRollingVel(self):
    #####################################################
        p = array(self.prev_vel)
        self.vit = p.mean()
     
    #####################################################
    def wheelCallback(self, msg):
    ######################################################
        enc = msg.data
#	rospy.logwarn(enc)
        if (enc < self.encoder_low_wrap and self.prev_encoder > self.encoder_high_wrap) :
            self.wheel_mult = self.wheel_mult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_encoder < self.encoder_low_wrap) :
            self.wheel_mult = self.wheel_mult - 1
           
        self.wheel_prev = self.wheel_latest
        #self.t_prec=self.t_latest

        self.wheel_latest = 1.0 * (enc + self.wheel_mult * (self.encoder_max - self.encoder_min)) 
        #self.t_latest_duration=rospy.Time.now()
        #self.t_latest=self.t_latest_duration.to_sec() 
        
        
#        rospy.logdebug("-D- %s wheelCallback msg.data= %0.3f wheel_latest = %0.3f mult=%0.3f" % (self.nodename, enc, self.wheel_latest, self.wheel_mult))

    ######################################################
    def encodCallback(self, msg):
    ######################################################
        self.time_past=self.time_actu        
        self.time_actu=msg.data
    
if __name__ == '__main__':
    """ main """
    pidVelocity = PidVelocity()
    pidVelocity.spin()
   
    
