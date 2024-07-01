
from ultralytics import YOLO, YOLOWorld
import abc
import cv2
import rospy
# from detector_node.msg import BboxMsg




class Detector(metaclass=abc.metaclass):
    def __init__(self):
        pass

    def get_bbox(self):
        pass
    def get_objects_points(self):
        pass


class YoloDetector(Detector):
    def __init__(self) -> None:
        self.model = YOLO("yolov8n.pt") 
        super(YoloDetector, self).__init__()

class YoloWorld(Detector):
    def __init__(self) -> None:
        self.model = YoloWorld()
        super(YoloWorld, self).__init__()

class OpenCVDetector(Detector):
    def __init__(self) -> None:
        super(OpenCVDetector, self).__init__()

    def color_detection(self) -> None :
        pass


    def get_bbox_depth(self):
        pass


def main():

    rospy.init_node('detector', anonymous=True)
    # Read parameters from the parameter server
    node_name = rospy.get_param('~node_name')
    topic_name = rospy.get_param('~topic_name')
    message_data = rospy.get_param('~message_data')
    message_number = rospy.get_param('~message_number')
    publish_rate = rospy.get_param('~publish_rate')

    detector = YoloDetector()


    yolo_publisher = rospy.Publisher('YoloWorld', BboxMsg, queue_size=10)
    opencv_publisher = rospy.Publisher("OpenCV", OpenCVMsg, queue_size=10)
    

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        custom_msg = BboxMsg()
        custom_msg.data = 
        custom_msg.number = 123

        rospy.loginfo(f"Publishing: {custom_msg}")
        pub.publish(custom_msg)
        rate.sleep()
