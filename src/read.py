#!/usr/bin/env python3

import rospy
from vision_node.msg import Detection
from vision_node.srv import EnableYolo, EnableYoloResponse
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import numpy as np
import tf.transformations as tft

# Global variable to enable/disable YOLO detection
yolo_enabled = False

# Function to handle the EnableYolo service
def handle_enable_yolo(req):
    global yolo_enabled
    yolo_enabled = req.enable
    return EnableYoloResponse(success=True)

# Function to read YAML configuration file
def read_yaml_file(file_path):
    with open(file_path, 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            rospy.logerr(exc)
            return None

# Initialize YOLO model
model = YOLO("yolov8n.pt")
bridge = CvBridge()
depth_image = None
camera_info = None
imu_pose = None

# Callback function for depth image data
def depth_image_callback(msg):
    global depth_image
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, "32FC1")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

# Callback function for camera info
def camera_info_callback(msg):
    global camera_info
    camera_info = msg

# Callback function for IMU PoseStamped data
def imu_pose_callback(msg):
    global imu_pose
    imu_pose = msg

# Transform a point from camera frame to global frame
def transform_to_global(point, pose):
    # Create transformation matrix from IMU pose
    translation = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
    rotation = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
    transform_matrix = tft.quaternion_matrix(rotation)
    transform_matrix[0:3, 3] = translation

    # Transform the point
    point_homogeneous = np.array([point.x, point.y, point.z, 1.0])
    point_global = np.dot(transform_matrix, point_homogeneous)

    return Point(point_global[0], point_global[1], point_global[2])

# Callback function for image data
def zed_image_callback(msg):
    global depth_image, camera_info, imu_pose, yolo_enabled
    if not yolo_enabled:
        return

    try:
        # Convert the ROS Image message to a numpy array
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Run YOLOv8 detection
        results = model.track(cv_image, persist=True)[0]

        for data in results.boxes.data:
            x_min, y_min, x_max, y_max, track_id, conf, cls = data.cpu().numpy()

            detection_msg = Detection()
            detection_msg.cls = int(cls)  # The class label of the detected object
            detection_msg.confidence = float(conf)  # The confidence score of the detection

            detection_msg.bounding_box = RegionOfInterest(
                x_offset=int(x_min),
                y_offset=int(y_min),
                height=int(y_max - y_min),
                width=int(x_max - x_min)
            )

            # Calculate the average depth within the bounding box
            if depth_image is not None:
                x_min_int = int(x_min)
                x_max_int = int(x_max)
                y_min_int = int(y_min)
                y_max_int = int(y_max)

                bbox_depth = depth_image[y_min_int:y_max_int, x_min_int:x_max_int]
                if bbox_depth.size > 0:
                    mean_depth = np.nanmean(bbox_depth)
                    if not np.isnan(mean_depth) and camera_info is not None:
                        # Convert depth to 3D point using camera intrinsic parameters
                        fx = camera_info.K[0]
                        fy = camera_info.K[4]
                        cx = camera_info.K[2]
                        cy = camera_info.K[5]

                        z = mean_depth
                        x_center = (x_min + x_max) / 2
                        y_center = (y_min + y_max) / 2
                        x = (x_center - cx) * z / fx
                        y = (y_center - cy) * z / fy

                        point_camera = Point(x=x, y=y, z=z)

                        # Transform the point to global coordinates if IMU pose is available
                        if imu_pose is not None:
                            point_global = transform_to_global(point_camera, imu_pose)
                            detection_msg.point = point_global
                        else:
                            detection_msg.point = point_camera

            pub.publish(detection_msg)

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

# Function to initialize subscribers
def initialize_subscribers(topics_file):
    topics_info = read_yaml_file(topics_file)
    if topics_info is None:
        rospy.logerr("Failed to read YAML file or file is empty.")
        return

    rospy.loginfo("YAML file read successfully.")
    rospy.Subscriber(topics_info['zed_camera']['image'], Image, zed_image_callback)
    rospy.Subscriber(topics_info['zed_camera']['depth_image'], Image, depth_image_callback)
    rospy.Subscriber(topics_info['zed_camera']['camera_info'], CameraInfo, camera_info_callback)
    rospy.Subscriber(topics_info['zed_camera']['imu_pose'], PoseStamped, imu_pose_callback)
    rospy.sleep(5)

if __name__ == "__main__":
    rospy.init_node('yolo_detector')

    # Publisher for Detection messages
    pub = rospy.Publisher('detections', Detection, queue_size=10)

    # Initialize the EnableYolo service
    service = rospy.Service('enable_yolo', EnableYolo, handle_enable_yolo)

    initialize_subscribers("params/topics.yml")

    rospy.spin()
