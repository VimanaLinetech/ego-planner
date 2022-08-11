#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from tf.transformations import quaternion_from_euler

class PositionCommandConverter:
    def __init__(self):
        rospy.init_node('planner2apm')

        self.poscommand_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=100)

        rospy.Subscriber('/planning/pos_cmd', PositionCommand, self.callback, tcp_nodelay=True)
        
        rospy.spin()

    def callback(self, apm_msg):

        poscommand = PoseStamped()

        poscommand.pose.position.x = apm_msg.position.x
        poscommand.pose.position.y = apm_msg.position.y
        poscommand.pose.position.z = apm_msg.position.z

        quaternion = quaternion_from_euler(0,0,apm_msg.yaw) # Roll, Pitch e Yaw

        poscommand.pose.orientation.x = quaternion[0]
        poscommand.pose.orientation.y = quaternion[1]
        poscommand.pose.orientation.z = quaternion[2]
        poscommand.pose.orientation.w = quaternion[3]

        poscommand.header.seq = apm_msg.header.seq
        poscommand.header.stamp = apm_msg.header.stamp
        poscommand.header.frame_id = 'map' 

        self.poscommand_pub.publish(poscommand)

if __name__ == '__main__':
    loop = PositionCommandConverter()