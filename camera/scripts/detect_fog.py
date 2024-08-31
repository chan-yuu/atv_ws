#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from car_interfaces.msg import CameraInterface

# cyun WIDC
# detect for fog by laplacian

# 初始化OpenCV视频捕捉
# cap = cv2.VideoCapture(0)  # 0表示默认摄像头，可以改为"/dev/video0"

# 初始化CvBridge
laplacian = 1000
bridge = CvBridge()

def detect_fog(image):
    '''回调函数在接收到ROS图像消息时调用，将ROS图像消息转换为OpenCV图像，并调用detect_fog函数进行雾天检测。如果检测到雾天，则发布消息。'''
    """检测图像是否为雾天"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    laplacian = cv2.Laplacian(gray, cv2.CV_64F).var()
    print("laplacian", laplacian)
    # 基于Laplacian方差的简单雾检测阈值
    if laplacian < 650:
        return True
    return False

def image_callback(ros_image):
    global laplacian
    try:
        # 将ROS图像消息转换为OpenCV图像
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    if detect_fog(cv_image):
        # fog_message = "Fog detected"
        fog_message = 1
        is_fog = 1
        rospy.loginfo(fog_message)
    else:
        fog_message = 0
        is_fog = 0
        rospy.loginfo(fog_message)

    msg = CameraInterface()
    msg.is_fog = is_fog
    msg.lap = laplacian
    pub.publish(msg) # 10 hz

if __name__ == '__main__':
    rospy.init_node('fog_detection_node', anonymous=True)

    # 订阅图像话题
    image_topic = "/camera/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)

    # 发布雾天检测信息
    pub = rospy.Publisher('decision_data_from_camera', CameraInterface, queue_size=10)

    rospy.loginfo("Fog detection node started")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down fog detection node")

    # # 释放摄像头资源
    # cap.release()
    # cv2.destroyAllWindows()

