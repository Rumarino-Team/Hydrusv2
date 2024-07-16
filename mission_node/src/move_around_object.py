#!/usr/bin/env python3
# This is the example state machine  for
# constructing the others machines with their respective 
# missions.

import smach
import rospy
from movement import UpdatePoseState, UpdatePoseToObjectState, ContinuePoseObjectMovement
from edge_cases import movement_edge_cases, movement_edge_case_callback
from geometry_msgs.msg import Pose, PoseStamped, Point
import os
"""
--------------------------------------------
DEFINE YOUR CUSTOM STATES IN HERE IF NECESSARY
--------------------------------------------
"""




class GoAroundObject(smach.StateMachine):
    def __init__(self, shared_state_data,object_name):
        smach.StateMachine.__init__(self, outcomes=['success', 'failure'])
        # shared_data is initialized inisde the initialize_subscribers() function
        # This is a global variable that is shared between all the states.
        self.userdata.shared_data = shared_state_data
        self.starting_point = shared_data.zed_data["pose"].pose.position
        
        # Implement a search for the name of the object e.g Path
        # Search for Point position 
        # Added to the Target Pose for giving it to the  StateMachine

        with self:
            smach.StateMachine.add('move_left_object',
                                    UpdatePoseToObjectState( edge_case_callback= movement_edge_cases,point= Point(x=-0.2,y = 0.2, z =0 ),
                                                            desired_object_name=object_name), 
                                    transitions={'success':'move_right_object', 'aborted':'failure', 'edge_case_detected':'failure', "object_not_detected": "move_left_object"})  
            smach.StateMachine.add("move_right_object", ContinuePoseObjectMovement(edge_case_callback=movement_edge_cases,point= Point(x=1,y=0,z=0)),
                                                                        transitions={'success':'move_starting_point', 'aborted':'failure', 'edge_case_detected':'failure'}
                                                                                   )

            smach.StateMachine.add("move_starting_point", UpdatePoseState(edge_case_callback=movement_edge_cases,point= self.starting_point),
                                    transitions={'success':'finish', 'aborted':'failure', 'edge_case_detected':'failure'})

# Running the state machine
if __name__ == '__main__':
    from subscribers import shared_data, initialize_subscribers
    rospy.init_node('your_state_machine_node')
    file_path = os.path.join(os.path.dirname(__file__), '../../configs/topics.yml')
    initialize_subscribers(file_path)
    object_name = rospy.get_param('object_name')
    sm = GoAroundObject(shared_data,object_name)
    outcome = sm.execute()