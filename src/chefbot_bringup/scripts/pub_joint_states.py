#! /usr/bin/env python
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_msgs.msg import  Header

pos_l=0
pos_r=0
vit_l=0
vit_r=0
eff_r=0
eff_l=0


def callback_vit_r(msg):
    global vit_r
    vit_r=msg.data
    return()
    
def callback_vit_l(msg):
    global vit_l
    vit_l=msg.data
    return()
    
def callback_pos_r(msg):
    global pos_r
    pos_r=msg.data
    return()
    
def callback_pos_l(msg):
    global pos_l
    pos_l=msg.data
    return()

rospy.init_node('joint_state_node')
pub_joint=rospy.Publisher('/joint_states',JointState,queue_size=10)

rate=rospy.Rate(10)
joint_msg=JointState()
joint_msg.name=["wheel_left_joint","wheel_right_joint"]
joint_msg.position=[pos_l,pos_r]
joint_msg.velocity=[vit_l,vit_r]
joint_msg.effort=[eff_l,eff_r]
"""
tf_buffer=tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tf_buffer)
"""

while not rospy.is_shutdown():
    
    rospy.Subscriber('wheel_vel_r',Float32,callback_vit_r)
    rospy.Subscriber('wheel_vel_l',Float32,callback_vit_l)
    rospy.Subscriber('pos_wheel_r',Float32,callback_pos_r)
    rospy.Subscriber('pos_wheel_l',Float32,callback_pos_l)
    
    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id="base_link"
    
    joint_msg.header = h
    
    joint_msg.name[0]="wheel_left_joint"
    joint_msg.name[1]="wheel_right_joint"
    joint_msg.position[0]=pos_l
    joint_msg.position[1]=pos_r
    joint_msg.velocity[0]=vit_l
    joint_msg.velocity[1]=vit_r
    joint_msg.effort[0]=eff_l
    joint_msg.effort[1]=eff_r
    
    
    pub_joint.publish(joint_msg)
    """
    if (tf_buffer.can_transform('base_footprint','odom', rospy.Time(),rospy.Duration(3.0))):        
        tf_buffer.lookup_transform('base_footprint','odom', rospy.Time.now(),rospy.Duration(3.0))
        rospy.loginfo('passe')
    """
    rate.sleep()
    