import rospy
import tensorflow as tf
from tensorflow.keras import Sequential, Dense
from std_msgs.msg import Float32
import os

rospy.init_node("ia_node")
pub = rospy.Publisher('decision', Float32)

rate = rospy.Rate(2)
decision = Float32()

model_path = '.'
if os.path.isfile(model_path):
    model = tf.keras.models.load_model('path/to/location')
else:
    model = Sequential()
    model.add(Dense(128, activation='relu'))
    model.add(Dense(64, activation='relu'))
    model.add(Dense(1, activation='tanh'))

    model.save('path/to/location')

#decision.data = x.
pub.publish(decision)