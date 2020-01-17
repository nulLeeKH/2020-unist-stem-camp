#!/usr/bin/env python

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


import numpy as np
import sys, math, random, copy
import rospy, copy, time
from sensor_msgs.msg import LaserScan
from racecar_ws.msg import drive_msg
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

AUTONOMOUS_MODE=False
count = 0

print("Complete to open Drive Node")

class PIDControl:
    def __init__(self):
        self.Kp = 6
        self.Ki = 0.0009
        self.Kd = 18

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

    def machine_learning_callback(self, msg):
        '''machine_learning_callback'''

    def drive_callback(self):
        '''Publishes drive commands'''
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

        self.drive_pub.publish(self.cmd)


if __name__ == "__main__":
    try:
        drvCalc = driveCalculator()
        PID = PIDControl()
        node = Drive()
        rospy.spin()
    except rospy.ROSInterruptException:
        exit()
