#!/usr/bin/env python3

"""
MIT License

Copyright (c) 2020 Kyung-ha Lee <i_am@nulleekh.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""


import rospy
from sensor_msgs.msg import Image

import cv2
import numpy as np
import tensorflow.keras

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

print("Complete to open Simple_camera Node")

USE_ML = True

if(USE_ML):
    np.set_printoptions(suppress=True)

    model = tensorflow.keras.models.load_model('/home/racecar/catkin_ws/src/racecar_ws/src/keras_model.h5')
    data = np.ndarray(shape=(1, 224, 224, 3), dtype=np.float32)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)


rospy.init_node('camera')
pub_im = rospy.Publisher('/camera', Image, queue_size=1)
ML_pub = rospy.Publisher('/teachable_machine', Float32MultiArray, queue_size=1)

while not rospy.is_shutdown():

    _, frame = cap.read()

    if frame is None:
        continue

    msg = Image()
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = 3 * msg.width
    msg.data = frame.flatten().tostring()

    pub_im.publish(msg)

    if(USE_ML):
    	image_array = cv2.resize(frame, (224, 224), interpolation = cv2.INTER_AREA)
    	normalized_image_array = image_array.astype(np.float32)/127.0 - 1
    	data[0] = normalized_image_array
    	prediction = model.predict(data)
    	#print(prediction)
    	prediction_data = Float32MultiArray()
    	prediction_data.data = np.array(prediction).flatten()
    	ML_pub.publish(prediction_data)




cap.release()

