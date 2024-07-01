

class Controller:
    def __init__():
        pass

class PbjecPoint:
    x = 0
    y = 0
    z = 0


class ControllerDepth(Controller):
    def __init__(self):
        pass

    def move(point: PbjecPoint):
        pass

class ControllerSurge(Controller):
    def __init__():
        pass

    def calculate_rotate(current_position, point_of_interest, current_orientation):
        return resulting_quaternion
    
    def helper_function_converritng_quaternions_to_degree():
        pass

    # Search for transformations



while rospy.shutdown():

    



def get_rotation_quaternion(current_position, point_of_interest, current_orientation):
    # Step 1: Calculate the vector to the point of interest
    vector_to_interest = np.array(point_of_interest) - np.array(current_position)
    
    # Step 2: Normalize the vector to get the direction
    direction_to_interest = vector_to_interest / np.linalg.norm(vector_to_interest)
    
    # Step 3: Determine the desired orientation
    # Assuming the object initially faces along the x-axis (1,0,0)
    initial_direction = np.array([1, 0, 0])
    
    # Step 4: Compute the rotation quaternion
    # Compute the axis of rotation (cross product of initial direction and target direction)
    axis_of_rotation = np.cross(initial_direction, direction_to_interest)
    
    # Compute the angle between the initial direction and the target direction (dot product)
    angle_of_rotation = np.arccos(np.dot(initial_direction, direction_to_interest))
    
    # Convert axis-angle to quaternion
    rotation_quaternion = tf.quaternion_about_axis(angle_of_rotation, axis_of_rotation)
    
    # Step 5: Combine with the current orientation
    # The resulting quaternion should be multiplied by the current orientation quaternion
    current_quaternion = tf.quaternion_from_euler(*current_orientation)
    resulting_quaternion = tf.quaternion_multiply(rotation_quaternion, current_quaternion)
    
    return resulting_quaternion

# Example usage
current_position = [0, 0, 0]
point_of_interest = [1, 1, 0]
current_orientation = [0, 0, 0]  # Assuming no initial rotation (Euler angles)

resulting_quaternion = get_rotation_quaternion(current_position, point_of_interest, current_orientation)
print("Resulting Quaternion: ", resulting_quaternion)