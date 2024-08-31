#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# cyun WIDC
# OPEN A CAMERA and get a ros image data

def main():
    rospy.init_node('camera_publisher_node', anonymous=True)
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()
    
    cap = cv2.VideoCapture(0)  # 打开默认相机，或者使用"/dev/video0"
    
    if not cap.isOpened():
        rospy.logerr("Cannot open camera")
        return
    
    rospy.loginfo("Camera publisher node started")

    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to capture image")
            continue
        
        try:
            image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            image_pub.publish(image_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
