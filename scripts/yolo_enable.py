#!/usr/bin/env python3

import rospy
from detector_node.srv import EnableYolo, EnableYoloRequest

def enable_yolo(enable):
    rospy.wait_for_service('enable_yolo')
    try:
        enable_yolo_service = rospy.ServiceProxy('enable_yolo', EnableYolo)
        req = EnableYoloRequest(enable=enable)
        resp = enable_yolo_service(req)
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False

if __name__ == '__main__':
    rospy.init_node('enable_yolo_client')
    enable = True  # Set to False to disable
   
