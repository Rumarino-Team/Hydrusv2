#!/usr/bin/env python

import math
import yaml
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, PoseStamped
from controller_node.srv import NavigateToWaypoint, NavigateToWaypointResponse

DEPTH_SPEED = 1
DEPTH_MOTORS_ID = [1, 2, 3, 4]  # front left, front right, back left, back right
ROTATION_SPEED = 1
FRONT_MOTORS_ID = [5, 6]  # left, right
BACK_MOTORS_ID = [7, 8]  # left, right
LINEAR_SPEED = 1
DELTA = 0.01

def read_yaml_file(file_path):
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        rospy.logerr("Failed to read YAML file: %s", str(e))
        return None

class SubController:
    def __init__(self):
        rospy.init_node('subcontroller', anonymous=True)

        self.initialize_subscribers('../../configs/topics.yml')

        self.service = rospy.Service('navigate_to_waypoint', NavigateToWaypoint, self.handle_navigate_request)
        self.target_pose = None
        self.current_pose = None
        self.detection = []
        self.thrusters_publishers = []
        self.thruster_values = [0] * 8
        self.moving = [False, False, False]  # [depth, rotation, linear]

        for i in range(1, 9):
            self.thrusters_publishers.append(rospy.Publisher('/thrusters/' + str(i) + '/', Vector3, queue_size=10))

        self.rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.target_pose is not None:
                self.move_submarine(self.current_pose, self.target_pose)

    def initialize_subscribers(self, topics_file):
        topics_info = read_yaml_file(topics_file)
        if topics_info is None:
            rospy.logerr("Failed to read YAML file or file is empty.")
            return

        rospy.loginfo("YAML file read successfully.")

        rospy.Subscriber(topics_info['zed_camera']['pose'], PoseStamped, self.zed_pose_callback)

    def zed_pose_callback(self, msg):
        self.current_pose = msg.pose

    def detection_callback(self, msg):
        self.detection = msg

    def move_submarine(self, current_pose, target_pose):
        self.calculate_thruster_values(current_pose, target_pose)
        for i in range(len(self.thruster_values)):
            self.thrusters_publishers[i].publish(self.thruster_values[i])
        self.rate.sleep()

    def calculate_thruster_values(self, current_pose, target_pose):
        if self.moving[0]:  # Go up or down
            if (current_pose.position.z - target_pose.position.z) > DELTA:
                for motor_id in DEPTH_MOTORS_ID:
                    self.thruster_values[motor_id] = DEPTH_SPEED
            elif (current_pose.position.z - target_pose.position.z) < DELTA:
                for motor_id in DEPTH_MOTORS_ID:
                    self.thruster_values[motor_id] = -DEPTH_SPEED
            else:
                self.moving = [False, True, False]
        elif self.moving[1]:  # Rotate
            current_yaw = self.quaternions_angle_difference(current_pose.position)
            target_yaw = self.quaternions_angle_difference(target_pose.position)
            angle_diff = target_yaw - current_yaw

            if angle_diff > DELTA:
                self.thruster_values[FRONT_MOTORS_ID[0]] = -ROTATION_SPEED
                self.thruster_values[FRONT_MOTORS_ID[1]] = ROTATION_SPEED
                self.thruster_values[BACK_MOTORS_ID[0]] = -ROTATION_SPEED
                self.thruster_values[BACK_MOTORS_ID[1]] = ROTATION_SPEED
            elif angle_diff < -DELTA:
                self.thruster_values[FRONT_MOTORS_ID[0]] = ROTATION_SPEED
                self.thruster_values[FRONT_MOTORS_ID[1]] = -ROTATION_SPEED
                self.thruster_values[BACK_MOTORS_ID[0]] = ROTATION_SPEED
                self.thruster_values[BACK_MOTORS_ID[1]] = -ROTATION_SPEED
            else:
                self.moving = [False, False, True]
        elif self.moving[2]:  # Move forward or backward
            if (current_pose.position.x - target_pose.position.x) > DELTA:
                for motor_id in FRONT_MOTORS_ID:
                    self.thruster_values[motor_id] = LINEAR_SPEED
                for motor_id in BACK_MOTORS_ID:
                    self.thruster_values[motor_id] = LINEAR_SPEED
            elif (current_pose.position.x - target_pose.position.x) < DELTA:
                for motor_id in FRONT_MOTORS_ID:
                    self.thruster_values[motor_id] = -LINEAR_SPEED
                for motor_id in BACK_MOTORS_ID:
                    self.thruster_values[motor_id] = -LINEAR_SPEED
            else:
                self.moving = [True, False, False]
        else:
            self.moving = [True, False, False]

    def quaternions_angle_difference(q1, q2):
        dot = q1.x*q2.x + q1.y*q2.y + q1.z*q2.z + q1.w*q2.w

        angle_difference = 2 * math.acos(dot)
        return angle_difference

    def handle_navigate_request(self, req):
        self.target_pose = req.target_pose
        return NavigateToWaypointResponse(success=True)

    def run(self):
        rospy.spin()  # Keep the service running

if __name__ == '__main__':
    try:
        controller = SubController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
