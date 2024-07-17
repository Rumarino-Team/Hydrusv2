import rospy
import smach
import random
from geometry_msgs.msg import Point
import math
from utils import quaternions_angle_difference, euler_to_quaternion
from controller_node.srv import NavigateToWaypoint, NavigateToWaypointRequest

class UpdatePoseState(smach.State):
    """Parameters:

    edge_case_callback - callback function for edge case detection
    point: None
    
    """
    def __init__(self, edge_case_callback, next_state_callback=None, point=None, threshold=1.2, stabilization_time=1,
                 outcomes=['success', 'edge_case_detected', 'aborted'], input_keys=['shared_data'], output_keys=['shared_data']):
        smach.State.__init__(self, outcomes=outcomes,
                             input_keys=input_keys,
                             output_keys=output_keys)
        self.edge_case_callback = edge_case_callback
        self.next_state_callback = next_state_callback
        self.point = point
        self.threshold = threshold
        self.stabilization_time = stabilization_time

    @staticmethod
    def pose_reached(current_pose, destination_point, threshold):
        # if not isinstance(current_pose, Point):
        #     rospy.logerr("current_pose must be an instance of Point and got of type: " + str(type(current_pose)))

        position_diff = math.sqrt(
                (current_pose.pose.position.x - destination_point.x) ** 2 +
                (current_pose.pose.position.y - destination_point.y) ** 2 +
                (current_pose.pose.position.z - destination_point.z) ** 2
            )

        return position_diff <= threshold 

    def call_movement(self, target_point):
        rospy.wait_for_service('navigate_to_waypoint')
        try:
            navigate_service = rospy.ServiceProxy('navigate_to_waypoint', NavigateToWaypoint)
            request = NavigateToWaypointRequest(target_point=target_point)
            response = navigate_service(request)
            rospy.loginfo("Service call successful: %s", response.success)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return 'aborted'

    def loop_monitor(self, userdata, target_point):
        shared_data = userdata.shared_data
        while not rospy.is_shutdown():
            if self.pose_reached(shared_data.zed_data["pose"], target_point, self.threshold):
                rospy.loginfo("Destination reached. Verifying stabilization.")
                stabilization_start = rospy.Time.now()
                while rospy.Time.now() - stabilization_start < rospy.Duration(0.5):
                    if not self.pose_reached(shared_data.zed_data["pose"], target_point, self.threshold):
                        break
                    rospy.sleep(0.1)
                else:  # If the loop completes without breaking
                    rospy.loginfo("Destination stabilized.")
                    return 'success'

            if False: #self.edge_case_callback(shared_data):
                rospy.logwarn("Edge case detected, transitioning to handle situation.")
                return 'edge_case_detected'

            rospy.sleep(0.1)
            print("Monitoring loop")
            
    def execute(self, userdata):
        self.call_movement(self.point)
        return self.loop_monitor(userdata, self.point)

class UpdatePoseToObjectState(UpdatePoseState):
    def __init__(self, desired_object_name, edge_case_callback=None, next_state_callback=None, point=None):
        super(UpdatePoseToObjectState, self).__init__(outcomes=['success', 'edge_case_detected', 'aborted', 'object_not_detected'],
                                                      input_keys=['shared_data'],
                                                      output_keys=['shared_data', 'detected_object'],
                                                      edge_case_callback=edge_case_callback,
                                                      next_state_callback=next_state_callback,
                                                      point=point)
        self.desired_object_name = desired_object_name

    def execute(self, userdata):
        shared_data = userdata.shared_data
        detections = shared_data.detector["box_detection"]
        for detection in detections.detections:
            if self.desired_object_name == detections.class_names[detection.cls]:
                userdata.detected_object = detection
                if self.point:  # Update the Pose with the Offset of a Point
                    target_point = Point(detection.point.x + self.point.x, detection.point.y + self.point.y, detection.point.z + self.point.z)
                    self.call_movement(target_point)
                else:
                    self.call_movement(detection.point)
                return self.loop_monitor(userdata, target_point)
        else:
            return "object_not_detected"

class ContinuePoseObjectMovement(UpdatePoseState):
    def __init__(self, offset_point, edge_case_callback=None, next_state_callback=None):
        super(ContinuePoseObjectMovement, self).__init__(outcomes=['success', 'edge_case_detected', 'aborted'],
                                                         input_keys=['shared_data', 'detected_object'],
                                                         output_keys=['edge_case'],
                                                         edge_case_callback=edge_case_callback,
                                                         next_state_callback=next_state_callback)
        self.offset_point = offset_point

    def execute(self, userdata):
        shared_data = userdata.shared_data
        detected_object = userdata.detected_object
        target_point = Point(detected_object.point.x + self.offset_point.x,
                             detected_object.point.y + self.offset_point.y,
                             detected_object.point.z + self.offset_point.z)
        self.call_movement(target_point)
        return self.loop_monitor(userdata,target_point)
