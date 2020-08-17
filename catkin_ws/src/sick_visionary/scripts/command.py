#!/usr/bin/env python

import argparse
import struct
import re
import getpass
import time
import Device
import CIDTypes

import rospy
from geometry_msgs.msg import Point

def talker():
    deviceControl = Device.Control('192.168.0.80',2122,'COLA_2',5)
    deviceControl.open()
    deviceControl.login(3, "client_09")
    print "connected and logged in to device"

    deviceControl.writeVariable('integrationTimeUs', struct.pack('>H',int(380)))          	#integrationTimeUs 380us
    deviceControl.writeVariable('averaging', struct.pack('>b', 1))                              # averaging Enable
    deviceControl.writeVariable('sendData', struct.pack('>b', 1))                               # sendData   Enable

    leftPocket = Point()
    rightPocket = Point()
    center = Point()

    pub = rospy.Publisher('chatter', Point, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(5) # 5hz
    rospy.loginfo('init ok')

    while not rospy.is_shutdown():
        detectionResult_serialized = deviceControl.readVariable(b"detectionResult")
        _, detectionResult = CIDTypes.DetectionResults.unpack(detectionResult_serialized)
        """print("Left pocket: ")
        print(detectionResult.leftPocket.X)
        print(detectionResult.leftPocket.Y)
        print(detectionResult.leftPocket.Z)
        print("Right pocket: ")
        print(detectionResult.rightPocket.X)
        print(detectionResult.rightPocket.Y)
        print(detectionResult.rightPocket.Z)
        print("Center: ")
        print(detectionResult.center.X)
        print(detectionResult.center.Y)
        print(detectionResult.center.Z)
        print('---')"""
        leftPocket.x = detectionResult.leftPocket.X
        leftPocket.y = detectionResult.leftPocket.Y
        leftPocket.z = detectionResult.leftPocket.Z
        rightPocket.x = detectionResult.rightPocket.X
        rightPocket.y = detectionResult.rightPocket.Y
        rightPocket.z = detectionResult.rightPocket.Z
        center.x = detectionResult.center.X
        center.y = detectionResult.center.Y
        center.z = detectionResult.center.Z

        pub.publish(leftPocket)
        pub.publish(rightPocket)
        pub.publish(center)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
