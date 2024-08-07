import rospy
import smach
from enum import Enum
import math

bouy_types  = Enum('Color', ['Abyddos', 'Earth'])
class CheckImageVisibleState(smach.State):
    def __init__(self, image_topic, desired_object_name):
        smach.State.__init__(self, outcomes=['undetected', 'detected', 'preempted'])
        self.image_data = None
        self.image_topic = image_topic
        self.desired_object_name = desired_object_name


        def execute(self, userdata):
            if self.image_data is not None:
                if self.desired_object_name in self.image_data.objects:
                    return 'detected'
                else:
                    return 'undetected'
            else:
                return 'preempted'


def BasicMovementEdgeCase(shared_data,**kwargs):
    for key, value in kwargs.items():
        print("{} -> {}".format(key, value)) 

def quaternions_angle_difference(q1, q2):
    """
    Calculate the angle between two quaternions
    """
    dot = q1.x*q2.x + q1.y*q2.y + q1.z*q2.z + q1.w*q2.w

    angle_difference = 2 * math.acos(dot)
    return angle_difference


def euler_to_quaternion(roll, pitch, yaw):
    # Calculate the sine and cosine of the half angles
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    # Compute the quaternion
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (w, x, y, z)