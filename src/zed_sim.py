#  Create a class for Emulating the zed
# Read the cameras connected  with opencv
# Publish the color rect and depth data
import rospy

def main():
    rospy.init_node('detector', anonymous=True)
    