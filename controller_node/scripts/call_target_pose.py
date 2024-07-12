#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from controller_node.srv import NavigateToWaypoint, NavigateToWaypointRequest

def call_navigate_service(target_pose):
    rospy.wait_for_service('navigate_to_waypoint')
    try:
        navigate_service = rospy.ServiceProxy('navigate_to_waypoint', NavigateToWaypoint)
        request = NavigateToWaypointRequest(target_pose=target_pose)
        response = navigate_service(request)
        rospy.loginfo("Service call successful: %s", response.success)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    rospy.init_node('navigate_to_waypoint_client')

    # Create a target pose
    target_pose = Pose()
    target_pose.position = Point(x=1.0, y=2.0, z=3.0)
    target_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    # Call the service
    call_navigate_service(target_pose)
