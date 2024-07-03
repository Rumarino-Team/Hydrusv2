#!/usr/bin/env python

import rospy
from detector_node.msg import Detection

def detection_callback(msg):
    rospy.loginfo("Received detection:")
    rospy.loginfo("Class: %d", msg.cls)
    rospy.loginfo("Confidence: %.2f", msg.confidence)
    rospy.loginfo("Bounding Box: x_offset=%d, y_offset=%d, height=%d, width=%d",
                  msg.bounding_box.x_offset, msg.bounding_box.y_offset,
                  msg.bounding_box.height, msg.bounding_box.width)
    rospy.loginfo("3D Point: x=%.2f, y=%.2f, z=%.2f",
                  msg.point.x, msg.point.y, msg.point.z)

def detection_listener():
    rospy.init_node('detection_listener', anonymous=True)
    rospy.Subscriber('detections', Detection, detection_callback)
    rospy.spin()

if __name__ == '__main__':
    detection_listener()
