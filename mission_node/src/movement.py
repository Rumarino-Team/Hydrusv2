import rospy
import smach
from geometry_msgs.msg import PoseWithCovariance 
import random
from std_msgs.msg import Time
from geometry_msgs.msg import Point
import math
import tf
from data import read_yaml_file
import os
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from utils import quaternions_angle_difference, euler_to_quaternion

class UpdatePoseState(smach.State):
    """Parameters:

    edge_case_callback - callback function for edge case detection
    point: None
    
    """
    def __init__(self,  edge_case_callback,next_state_callback = None , point = None ,threshold = 1.2,
                        angle_threshold = 0.04):
        smach.State.__init__(self, outcomes=['success', 'edge_case_detected', 'aborpose_reachedted'],
                             input_keys=['shared_data'],
                             output_keys=['shared_data'])
        self.edge_case_callback = edge_case_callback
        self.next_state_callback = next_state_callback
        self.point = point
        self.threshold = threshold
        self.angle_threshold = angle_threshold
        self.init_waypoint_set_service = rospy.ServiceProxy()

    @staticmethod
    def generate_waypoints(num_waypoints):
        waypoints = []
        for _ in range(num_waypoints):
            waypoint = Waypoint()
            waypoint.point = Point(random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-20, -10))
            waypoint.max_forward_speed = random.uniform(0, 5)
            waypoint.heading_offset = random.uniform(-3.14, 3.14)
            waypoint.use_fixed_heading = random.choice([True, False])
            waypoint.radius_of_acceptance = random.uniform(0, 1)
            waypoints.append(waypoint)
        return waypoints
    @staticmethod
    def WaypointFromPose(pose, speed, heading_offset, fixed_heading, radius_of_acceptance):
        waypoints = []
        waypoint = Waypoint()
        waypoint.point = pose.point
        waypoint.max_forward_speed = speed
        waypoint.heading_offset = heading_offset
        waypoint.use_fixed_heading = fixed_heading
        waypoint.radius_of_acceptance = radius_of_acceptance
        waypoints.append(waypoint)
        return waypoints
    

    @staticmethod
    def pose_reached( current_pose, destination_pose, threshold, angle_threshold):
        # Check if the current pose is within a certain threshold of the destination pose
        # The function 'compare_poses' should return True if the poses are similar within the threshold
        # print("current_pose",current_pose)
        print(destination_pose)

        if not isinstance(current_pose, Pose):
            rospy.logerr("current_pose must be an instance of Pose and got of type: " + str(type(current_pose)))

        if not isinstance(destination_pose, Pose):
            rospy.logerr("destination_pose must be an instance of Pose and got of type: " + str(type(destination_pose)))

     
        position_diff = math.sqrt(
                (current_pose.position.x - destination_pose.position.x) ** 2 +
                (current_pose.position.y - destination_pose.position.y) ** 2 +
                (current_pose.position.z - destination_pose.position.z) ** 2
            )
        
        
        yaw_angle_diff = quaternions_angle_difference(current_pose.pose.pose.orientation, destination_pose.orientation)

        return position_diff <= threshold and yaw_angle_diff <= angle_threshold #and orientation_diff <= threshold


    def call_movement(self, waypoints):
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
            target_poses = [Pose(point = waypoint.point, orientation = Quaternion(*euler_to_quaternion(0,0,self.heading_offset) )) for waypoint in waypoints]
            while not rospy.is_shutdown() and rospy.Time.now() - start_time < self.timeout_duration:
                if self.pose_reached(shared_data.submarine_pose, target_poses[0], self.threshold):
                    rospy.loginfo("Destination reached. Verifying stabilization.")
                    # Ensure stabilization for the configured time
                    stabilization_start = rospy.Time.now()
                    while rospy.Time.now() - stabilization_start < rospy.Duration(0.5):
                        if not self.pose_reached(shared_data.submarine_pose, target_poses[0], self.threshold):
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
        for object in  shared_data.zed_data["ObjectsStamped"]:
            if self.desired_object_name == object.label:
                self.object_data = object

        if self.object_data:
            object_waypoint = self.zedObject2Waypoint()
            self.waypoints.append(object_waypoint)

        else:
            # Failed to identify object
            return "object_not_detected"

        self.call_movement()
        return self.loop_monitor(userdata, self.waypoints)

