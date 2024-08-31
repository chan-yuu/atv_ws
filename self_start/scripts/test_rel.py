#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Int32
import json

received_message = False  # 用于标记是否接收到消息

def callback(data):
    global received_message
    json_data = data.data
    data = json.loads(json_data)

    # 从解析后的数据中获取消息内容
    message = data['message']
    if message == "Hello, world!":
        # rospy.loginfo("Received message: %s", message)
        received_message = True
    else:
        received_message = False

def publish_status():
    global received_message
    # 初始化ROS节点
    rospy.init_node('string_subscriber_node', anonymous=True)

    # 创建一个发布者，发布整型消息到名为 "status_topic" 的话题
    pub = rospy.Publisher('status_topic', Int32, queue_size=10)

    rate = rospy.Rate(10)  # 设置循环的频率为1Hz

    while not rospy.is_shutdown():
        # if rospy.is_shutdown() or not rospy.get_param('/string_publisher_node/running', False):
        #     # 如果节点被关闭或者 "string_publisher_node" 停止运行，发送消息 1
        #     rospy.loginfo("Sending message: 1")
            # pub.publish(1)
        if received_message:
            # wirecontrol被一键关闭或者ctrl+c那就回复自动驾驶
            # 如果接收到 "Hello, world!" 消息，发送消息 2
            # rospy.loginfo("Sending message: 2")
            print('\033[32m' + "Sending message: 2" + '\033[0m')
            pub.publish(2)
            received_message = False
        else:
            # wirecontrol启动着的则不再发送
            pub.publish(1)
            # rospy.loginfo("Sending message: 1")
            print('\033[33m' + "Sending message: 1" + '\033[0m')

        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.Subscriber('string_topic', String, callback)

        publish_status()
    except rospy.ROSInterruptException:
        pass