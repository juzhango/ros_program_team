#!/usr/bin/env python

import argparse
import struct
import re
import getpass
import time
import Device
import CIDTypes

import rospy
#from sensor_msgs.msg import PointCloud
#from geometry_msgs.msg import Polygon
#from geometry_msgs.msg import Point32
#from geometry_msgs.msg import Point
#from geometry_msgs.msg import PoseStamped
#from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu

def talker():

    strIP = rospy.get_param('/talker/sick_visionary_ip')                                   # DeviceIpAddr
    port = rospy.get_param('/talker/sick_visionary_port')                                # DeviceIpPort
    integrationTimeUs = rospy.get_param('/talker/intTimeUs')                      # integrationTimeUs
    mountingTilt  = rospy.get_param('/talker/mountingTilt')                           # Tilt

    deviceControl = Device.Control(strIP,port,'COLA_2',5)
    deviceControl.open()
    deviceControl.login(3, "client_09")
    print "connected and logged in to device"
    deviceControl.writeVariable('integrationTimeUs', struct.pack('>H',int(integrationTimeUs)))
    deviceControl.writeVariable('averaging', struct.pack('>b', True))                                                                   # averaging Enable
    deviceControl.writeVariable('mountingTilt',struct.pack('>f',float(mountingTilt)))
    deviceControl.writeVariable('sendData', struct.pack('>b', True))                                                                   # sendData   Enable

    sick_msg = Imu()
    sick_msg.header.seq = 0
    pub = rospy.Publisher('/talker/chatter', Imu, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(5) # 5hz
    rospy.loginfo('init ok')
    rospy.loginfo('orientation:center    angular_velocity:leftPocket    linear_acceleration:rightPocket')

    while not rospy.is_shutdown():
        detectionResult_serialized = deviceControl.readVariable(b"detectionResult")
        _, detectionResult = CIDTypes.DetectionResults.unpack(detectionResult_serialized)
        '''print("Left pocket: ")
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
        print('---')'''
        sick_msg.header.seq = sick_msg.header.seq + 1
        sick_msg.header.stamp = rospy.Time.now()
        sick_msg.header.frame_id = 'sick_visionary'

        sick_msg.orientation.x = detectionResult.center.X
        sick_msg.orientation.y = detectionResult.center.Y
        sick_msg.orientation.z = detectionResult.center.Z

        sick_msg.angular_velocity.x = detectionResult.leftPocket.X
        sick_msg.angular_velocity.y = detectionResult.leftPocket.Y
        sick_msg.angular_velocity.z = detectionResult.leftPocket.Z

        sick_msg.linear_acceleration.x = detectionResult.rightPocket.X
        sick_msg.linear_acceleration.y = detectionResult.rightPocket.Y
        sick_msg.linear_acceleration.z = detectionResult.rightPocket.Z

        pub.publish(sick_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
