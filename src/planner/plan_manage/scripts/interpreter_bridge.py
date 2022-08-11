#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import StatusText
from geometry_msgs.msg import PoseStamped
from tf.transformations import  quaternion_from_euler
import math

class InterpreterBridge:
    def __init__(self):
        rospy.init_node('onboard-offboard-interpreter')

        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=0.1)

        rospy.Subscriber("/mavros/statustext/recv", StatusText, self.callback, tcp_nodelay=True)

        self.callback()

    def callback(self, msg):
        
        position = msg.text 

        position.split(" ")
        
        ego_planner = PoseStamped()

        # Ego Planner publisher
        
        yaw_in_deg = position[3]
        yaw_in_rad = (yaw_in_deg*math.pi)/180

        quat = quaternion_from_euler (0, 0, yaw_in_rad)
        
        ego_planner.pose.position.x = float(position[0])
        ego_planner.pose.position.y = float(position[1])
        ego_planner.pose.position.z = float(position[2])

        ego_planner.pose.orientation.x = quat[0]
        ego_planner.pose.orientation.y = quat[1]
        ego_planner.pose.orientation.z = quat[2]
        ego_planner.pose.orientation.w = quat[3]

        ego_planner.header.stamp = rospy.Time.now() 
        ego_planner.header.frame_id = 'map'         

        self.pub.publish(ego_planner)
        
if __name__ == '__main__':
    loop = InterpreterBridge()
