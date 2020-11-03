#!/usr/bin/env python

import rospy
import numpy as np
import tensorflow as tf
from tensorflow.keras import Sequential
from tensorflow.keras.layers import Dense
from std_msgs.msg import Float32
import os

def callback(msg):
    decision.data = model(np.array([[msg.data]]))[0]
    print(decision.data)
    pub.publish(decision)

rospy.init_node("ia_node")

rate = rospy.Rate(2)
decision = Float32()

model_path = 'my_model'
if os.path.isfile(model_path):
    model = tf.keras.models.load_model(model_path)
else:
    model = Sequential()
    model.add(Dense(128, activation='relu'))
    model.add(Dense(64, activation='relu'))
    model.add(Dense(1, activation='tanh'))

    optimizer = tf.keras.optimizers.RMSprop(0.001)
    model.compile(loss='mean_squared_error',
                  optimizer=optimizer,
                  metrics=['mean_absolute_error', 'mean_squared_error'])

    model.save(model_path)

pub = rospy.Publisher('/decision', Float32)
sub = rospy.Subscriber('/counter', Float32, callback)
rospy.spin()
