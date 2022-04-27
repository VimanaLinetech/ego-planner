#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class OdometryConverter:
    def __init__(self):
        rospy.init_node('odom_pose_converter')

        odom_topic = rospy.get_param('~odom_topic', '/rtabmap/rtabmap/odom')
        pose_pub_topic = rospy.get_param('~pose_pub_topic', '/camera/pose')

        self.pose_pub = rospy.Publisher(pose_pub_topic, PoseStamped, queue_size=10)

        rospy.Subscriber(odom_topic, Odometry, self.callback, tcp_nodelay=True)
        
        rospy.spin()

    def callback(self, msg):
        
        # Creating messages
        # Odometry --> PoseStamped

        pub_msg = PoseStamped()

        pub_msg.pose.position.x = msg.pose.pose.position.x
        pub_msg.pose.position.y = msg.pose.pose.position.y
        pub_msg.pose.position.z = msg.pose.pose.position.z

        pub_msg.pose.orientation.x = msg.pose.pose.orientation.x
        pub_msg.pose.orientation.y = msg.pose.pose.orientation.y
        pub_msg.pose.orientation.z = msg.pose.pose.orientation.z
        pub_msg.pose.orientation.w = msg.pose.pose.orientation.w

        pub_msg.header.seq = msg.header.seq
        pub_msg.header.stamp = msg.header.stamp
        pub_msg.header.frame_id = 'map' 

        self.pose_pub.publish(pub_msg)

if __name__ == '__main__':
    loop = OdometryConverter()