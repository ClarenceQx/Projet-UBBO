#! /usr/bin/env python
import rospy

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import  Header
from geometry_msgs.msg import Quaternion
#Importing ROS data type for IMU

from tf.transformations import quaternion_from_euler


#init variables imu

lin_accel_x=0
lin_accel_y=0
lin_accel_z=0
quat_x=0
quat_y=0
quat_z=0
quat_w=1
calib_gyro=0
calib_accel=0
calib_syst=0
etat_calib=0
roll=0
pitch=0
yaw=0
d=1


def callback_imu(msg):
    
    global lin_accel_x
    global lin_accel_y
    global lin_accel_z
    global quat_x
    global quat_y
    global quat_z
    global quat_w
    global roll
    global pitch
    global yaw    
    global calib_gyro
    global calib_accel
    global calib_syst
    global etat_calib
    
    lin_accel_x=msg.data[0]
    lin_accel_y=msg.data[1]
    lin_accel_z=msg.data[2]
    roll=msg.data[3]
    pitch=msg.data[4]
    yaw=msg.data[5]
    calib_gyro=msg.data[6]
    calib_accel=msg.data[7]
    calib_syst=msg.data[8]
    
    if (calib_gyro==3 and etat_calib==0):
        rospy.loginfo("gyro calibrated %f", calib_gyro)
        etat_calib=1
    if (calib_accel==3 and etat_calib==1):
        rospy.loginfo("accel calibrated %f", calib_accel)
        etat_calib=2
    if (calib_syst==3 and etat_calib==2):
        rospy.loginfo("syst calibrated %f", calib_syst)
        etat_calib=3
        rospy.logwarn("imu ready")
    
    q = quaternion_from_euler(roll,pitch,yaw)
    
    quat_x=q[0]
    quat_y=q[1]
    quat_z=q[2]
    quat_w=q[3]
    
    
    """
    quat_x=msg.data[3]
    quat_y=msg.data[4]
    quat_z=msg.data[5]
    quat_w=msg.data[6]
    test_calib=[msg.data[7],msg.data[8],msg.data[9]]
    """
    
    return()


# subcribe imu

rospy.init_node('imu_node')
pub_imu=rospy.Publisher('/imu_data',Imu,queue_size=10)

rate=rospy.Rate(2)
imu_msg=Imu()

while not rospy.is_shutdown():
    
    encodeurs=rospy.Subscriber('info_imu',Float32MultiArray,callback_imu)
    #base_frame_id=rospy.get_param('~base_frame_id','gyro_link') 
    
    h = Header()
    h.stamp = rospy.Time.now()
    #h.frame_id = base_frame_id
    h.frame_id="base_link"
    
    imu_msg.header = h

    imu_msg.orientation_covariance = [1000000.0,0.0,0.0,0.0,1000000.0,0.0,0.0,0.0,0.05]
    imu_msg.angular_velocity_covariance = [1000000.0,0.0,0.0,0.0,1000000.0,0.0,0.0,0.0,0.05]
    #imu_msg.linear_acceleration_covariance = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    imu_msg.linear_acceleration_covariance = [1000000.0,0.0,0.0,0.0,1000000.0,0.0,0.0,0.0,0.0]

    #imu_msg.orientation=Quaternion(quat_x,quat_y,quat_z,quat_w)
    
    #d=quat_x+quat_x+quat_y*quat_y+quat_z*quat_z+quat_w*quat_w
    
        
    imu_msg.orientation.x = quat_x
    imu_msg.orientation.y = quat_y
    imu_msg.orientation.z = quat_z
    imu_msg.orientation.w = quat_w
    
    imu_msg.angular_velocity.x=0.0
    imu_msg.angular_velocity.y=0.0
    imu_msg.angular_velocity.z=0.0
    
    imu_msg.linear_acceleration.x=lin_accel_x
    imu_msg.linear_acceleration.y=lin_accel_y
    imu_msg.linear_acceleration.z=lin_accel_z
    
    pub_imu.publish(imu_msg)
    
    rate.sleep()
    