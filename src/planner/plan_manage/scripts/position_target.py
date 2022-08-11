#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import StatusText

class OdometryConverter:
    def __init__(self):
        rospy.init_node('onboard-offboard-broadcaster')

        self.pose_pub = rospy.Publisher("/mavros/statustext/send", StatusText, queue_size=10)

        self.callback()

    def callback(self):
        
        x,y,z,yaw = input("Place x y z yaw: ").split()

        poscommand = StatusText()

        poscommand.text = str(x+" "+y+" "+z+" "+yaw)

        poscommand.severity = 7

        poscommand.header.stamp = rospy.Time.now() 
        poscommand.header.frame_id = 'map' 

        self.pose_pub.publish(poscommand)

if __name__ == '__main__':
    loop = OdometryConverter()
