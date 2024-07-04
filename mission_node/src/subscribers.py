import yaml
import rospy
from sensor_msgs.msg import Imu, Image, CameraInfo
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from zed_interfaces.msg import ObjectsStamped, RGBDSensors
from nav_sensors.msg import DVL_MSG
from rospy.exceptions import ROSException



def read_yaml_file(file_path):
    with open(file_path, 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            rospy.logerr(exc)
            return None

# Class for accesing the Robots Data in the state machine
class SharedData:
    def __init__(self):
        self.zed_data = {
            "imu" : None,
            "pose" : None,
            "odom": None,
            "path_odom": None
        }
        self.detector = {
            :
        }


# Global variable for accesing the shared_data 
shared_data = SharedData()

###########################################
    # Zed Camera Callbacks
def zed_imu_callback(msg):
    shared_data.zed_data['imu'] = msg

def zed_pose_callback(msg):
    shared_data.zed_data['pose'] = msg

def zed_odom_callback(msg):
    shared_data.zed_data['odom'] = msg

def zed_path_odom_callback(msg):
    shared_data.zed_data['path_odom'] = msg
#############################
    # Calculated Pose
#Either Simulation or  real IMU calculation or Zed_Camera pose
def pose_callback(msg):
    shared_data.submarine_pose = msg

###############################




# Initialize subscribers
def initialize_subscribers(topics_file):
    topics_info = read_yaml_file(topics_file)
    if topics_info is None:
        rospy.logerr("Failed to read YAML file or file is empty.")
        return

    rospy.loginfo("YAML file read successfully.")

    # Initialize Subscribers
    rospy.Subscriber(topics_info['zed_camera']['objects_stamped'], ObjectsStamped, zed_objects_callback)
    rospy.Subscriber(topics_info['zed_camera']['camera_info'], CameraInfo, zed_camera_info_callback)

    rospy.Subscriber(topics_info['submarine_pose']['pose'], Odometry, pose_callback)

    #wait 5 seconds for the subscribers to get the first message
    rospy.sleep(5)
    # Check if any of the subscribed data is still None
    data_sources = [
        ('zed_pose', shared_data.zed_data['pose']),
        ('zed_odom', shared_data.zed_data['odom']),
        ('zed_path_odom', shared_data.zed_data['path_odom']),
        ('zed_path_map', shared_data.zed_data['path_map']),
        ('submarine_pose', shared_data.submarine_pose),
    ]
 
    none_sources = [name for name, value in data_sources if value is None]
    if none_sources:
        missing_data_info = ", ".join(none_sources)
        rospy.logerr("The following data sources are still None: {0}".format(missing_data_info))
        raise ROSException("Not all required data sources are providing data.")
    else:
        rospy.loginfo("All subscribed data sources are providing data.")