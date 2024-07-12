#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, PoseStamped

class SubListener:
    def __init__(self):
        rospy.init_node('sub_listener', anonymous=True)
        
        # Subscribers to the same topics published by SubController
        rospy.Subscriber('/thrusters/1', Vector3, self.thruster_callback)
        rospy.Subscriber('/thrusters/2', Vector3, self.thruster_callback)
        rospy.Subscriber('/thrusters/3', Vector3, self.thruster_callback)
        rospy.Subscriber('/thrusters/4', Vector3, self.thruster_callback)
        rospy.Subscriber('/thrusters/5', Vector3, self.thruster_callback)
        rospy.Subscriber('/thrusters/6', Vector3, self.thruster_callback)
        rospy.Subscriber('/thrusters/7', Vector3, self.thruster_callback)
        rospy.Subscriber('/thrusters/8', Vector3, self.thruster_callback)
        
        
        rospy.spin()  # Keep the subscriber running
    
    def thruster_callback(self, msg):
        rospy.loginfo("Thruster command received: %s", msg)

    def pose_callback(self, msg):
        rospy.loginfo("Pose received: %s", msg)

    def odom_callback(self, msg):
        rospy.loginfo("Odometry received: %s", msg)

    def objects_callback(self, msg):
        rospy.loginfo("Objects received: %s", msg)

if __name__ == '__main__':
    try:
        listener = SubListener()
    except rospy.ROSInterruptException:
        pass
