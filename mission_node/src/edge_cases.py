



#  Edge cases are bool function that given the Shared Data will return True
# if a Edge case has been detected or False if everything is okay.
import numpy as np
import cv2


movement_edge_cases = [
    "on_the_surface_of_the_pool_and_waypoint_is_not_reachable",
    "very_close_to_the_wall",
    "not_enough_baterry_to_reach_waypoint",
    ""
]

def movement_edge_case_callback(shared_data,waypoint, config_data):
    if shared_data.dvl_data.depth >= config_data['pool_depth']: # Check if depth is on the limit of the pool surface. 
        if waypoint.point.z > config_data['pool_depth']: # Check if waypoint is above surface
            return True

    