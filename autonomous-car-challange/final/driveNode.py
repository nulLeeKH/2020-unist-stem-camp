#!/usr/bin/env python

import numpy as np
import sys, math, random, copy
import rospy, copy, time
from sensor_msgs.msg import LaserScan
from racecar_ws.msg import drive_msg
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
from color_segmentation import cd_color_segmentation

AUTONOMOUS_MODE=False
count = 0

print("Complete to open Drive Node")

class PIDControl:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.p = 0
        self.i = 0
        self.d = 0

        self.privErr = 0

        self.cnt = 0

    def PIDCalc(self, error, interval):
        self.p = error * self.Kp
        self.i += error * self.Ki * interval
        self.d = (error-self.privErr) * self.Kd
        self.privErr = error

        return self.p + self.i + self.d

class driveCalculator():
    def findLeast(self, value):
        least = 10
        for i in range(len(value)):
            if value[i] != 0:
                if value[i]<least:
                    least = value[i]
        if least == 10:
            least = 0
        return least

class Drive:
    def __init__(self):
        rospy.init_node("potentialField")
        self.data = None
        self.flag_box = ((0,0),(0,0))
        self.bridge = CvBridge()
        self.cmd = drive_msg()
        self.camera_sub = rospy.Subscriber("/camera", Image, self.camera_callback)
        self.ML_sub = rospy.Subscriber("/teachable_machine", Float32MultiArray, self.machine_learning_callback)

        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.drive_pub = rospy.Publisher("/drive", drive_msg, queue_size=1)

        self.cartPoints = [None for x in range(500)]
        self.finalVector = [100, 0]
        self.max= 255

    def scan_callback(self, data):
        '''Checks LIDAR data'''
        self.data = data.ranges
        self.drive_callback()

    def camera_callback(self, msg):
        '''camera_callback'''
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.flag_box = cd_color_segmentation(self.cv_image, show_image=False)

        self.drive_callback()

    def machine_learning_callback(self, msg):
        '''machine_learning_callback'''

    def drive_callback(self):
        '''Publishes drive commands'''
    	if self.flag_box == ((0,0),(0,0)):
    		left = (drvCalc.findLeast(self.data[34:100]) - 0.1) * 100
    		right = (drvCalc.findLeast(self.data[399:465]) - 0.1) * 100

    		front = (drvCalc.findLeast(self.data[0:34] + self.data[465:500]) - 0.15) * 100

    		PIDAngle = PID.PIDCalc(left-right, 0.01)

    		if PIDAngle > 255:
    		    PIDAngle = 255
    		elif PIDAngle < -255:
    		    PIDAngle = -255


    		self.cmd.velocity = 255

    		self.cmd.drive_angle = PIDAngle
    	else:
    		error = 320 - (self.flag_box[0][0] + self.flag_box[1][0]) / 2

    		PIDAngle = linePID.PIDCalc(error, 0.01)

    		if PIDAngle > 255:
    		    PIDAngle = 255
    		elif PIDAngle < -255:
    		    PIDAngle = -255


    		self.cmd.velocity = 255

    		self.cmd.drive_angle = PIDAngle



        self.drive_pub.publish(self.cmd)


if __name__ == "__main__":
    try:
        drvCalc = driveCalculator()
        PID = PIDControl(9, 0.0009, 33)
	linePID = PIDControl(3, 0.0009, 9)
        node = Drive()
        rospy.spin()
    except rospy.ROSInterruptException:
        exit()

