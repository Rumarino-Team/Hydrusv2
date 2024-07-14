# This is the example state machine  for
# constructing the others machines with their respective 
# missions.

import smach
import rospy
from movement import UpdatePoseState, UpdatePoseToObjectState
from edge_cases import movement_edge_cases, movement_edge_case_callback
from geometry_msgs.msg import Pose, PoseStamped, Point
import os
"""
--------------------------------------------
DEFINE YOUR CUSTOM STATES IN HERE IF NECESSARY
--------------------------------------------
"""
# Define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failure'])

    def execute(self, userdata):
        print("Try Again to detect object")
        return "failure"
    




class YourStateMachine(smach.StateMachine):
    def __init__(self, shared__state_data):
        smach.StateMachine.__init__(self, outcomes=['success', 'failure'])
        # shared_data is initialized inisde the initialize_subscribers() function
        # This is a global variable that is shared between all the states.
        self.userdata.shared_data = shared__state_data

        # Implement a search for the name of the object e.g Path
        # Search for Point position 
        # Added to the Target Pose for giving it to the  StateMachine

        with self:
            smach.StateMachine.add('Move_to_Buoy',
                                    UpdatePoseToObjectState( edge_case_callback= movement_edge_cases,
                                                            desired_object_name= "Gate"), 
                                    transitions={'success':'success', 'aborted':'failure', 'edge_case_detected':'failure'})


# Running the state machine
if __name__ == '__main__':
    from subscribers import shared_data, initialize_subscribers
    rospy.init_node('your_state_machine_node')
    file_path = os.path.join(os.path.dirname(__file__), '../../config/topics.yaml')
    initialize_subscribers(file_path)
    sm = YourStateMachine(shared_data)
    outcome = sm.execute()