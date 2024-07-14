import rospy
import smach
import random
from geometry_msgs.msg import Point
import math
import os
from utils import quaternions_angle_difference, euler_to_quaternion

class UpdatePoseState(smach.State):
    """Parameters:

    edge_case_callback - callback function for edge case detection
    point: None
    
    """
    def __init__(self,  edge_case_callback,next_state_callback = None , point = None ,threshold = 1.2, stabilization_time = 1):
        smach.State.__init__(self, outcomes=['success', 'edge_case_detected', 'aborpose_reachedted'],
                             input_keys=['shared_data'],
                             output_keys=['shared_data'])
        self.edge_case_callback = edge_case_callback
        self.next_state_callback = next_state_callback
        self.point = point
        self.threshold = threshold
        self.stabilization_time = stabilization_time
        self.init_waypoint_set_service = rospy.ServiceProxy()



    @staticmethod
    def pose_reached( current_pose, destination_point, threshold):
        # Check if the current pose is within a certain threshold of the destination pose
        # The function 'compare_poses' should return True if the poses are similar within the threshold
        # print("current_pose",current_pose)

        if not isinstance(current_pose, Pose):
            rospy.logerr("current_pose must be an instance of Pose and got of type: " + str(type(current_pose)))

     
        position_diff = math.sqrt(
                (current_pose.position.x - destination_point.position.x) ** 2 +
                (current_pose.position.y - destination_point.position.y) ** 2 +
                (current_pose.position.z - destination_point.position.z) ** 2
            )
        

        return position_diff <= threshold 


    def call_movement(self):
        # Call InitWaypointSet service
        try:
            if not response.success:
                rospy.logerr("Failed to initiate InitWaypointSet service.")
                return 'aborted'
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return 'aborted'

    def loop_monitor(self, userdata, waypoints):
            shared_data = userdata.shared_data
            start_time = rospy.Time.now()  # Start time for timeout calculation
            # For the target poses we only change the yaw orientation into the quaternion
            while not rospy.is_shutdown() and rospy.Time.now() - start_time < self.timeout_duration:
                if self.pose_reached(shared_data.zed_data["pose"], shared_data.detector["box_detection"], self.threshold):
                    rospy.loginfo("Destination reached. Verifying stabilization.")
                    # Ensure stabilization for the configured time
                    stabilization_start = rospy.Time.now()
                    while rospy.Time.now() - stabilization_start < rospy.Duration(0.5):
                        if not self.pose_reached(shared_data.zed_data["pose"], shared_data.detector["box_detection"], self.threshold):
                            break
                        rospy.sleep(0.1)
                    else:  # If the loop completes without breaking
                        rospy.loginfo("Destination stabilized.")
                        return 'success'

                if self.edge_case_callback(shared_data):
                    rospy.logwarn("Edge case detected, transitioning to handle situation.")
                    return "edge_case_detected"

                rospy.sleep(0.1)
                print("Monitoring loop")

            rospy.loginfo("Failed to reach destination within the timeout.")
            return 'timeout'  # or 'failed' based on your terminology

class UpdatePoseToObjectState(UpdatePoseState):
    def __init__(self, desired_object_name, edge_case_callback,next_state_callback ):
        super(UpdatePoseToObjectState, self).__init__(outcomes=['success', 'edge_case_detected', 'aborted', "object_not_detected"],
                                                      input_keys=['shared_data'],
                                                      output_keys=['edge_case'],
                                                 edge_case_callback=edge_case_callback,
                                                   next_state_callback=next_state_callback)
        self.desired_object_name = desired_object_name
        self.object_data = None


    def execute(self, userdata):
        shared_data = userdata.shared_data
        for detection in  shared_data.detector["box_detection"]:
            if self.desired_object_name ==  detection.:
                self.object_data = object

        else:
            # Failed to identify object
            return "object_not_detected"

        self.call_movement()
        return self.loop_monitor(userdata)

