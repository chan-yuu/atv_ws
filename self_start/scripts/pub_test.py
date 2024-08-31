#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import json



def publish_message():
    # 初始化ROS节点
    rospy.init_node('string_publisher_node', anonymous=False)

    # 创建一个发布者，发布String类型的消息到名为 "string_topic" 的话题
    pub = rospy.Publisher('string_topic', String, queue_size=10)

    # 设置循环的频率
    rate = rospy.Rate(10)  # 1Hz，即每秒发布一次消息

    while not rospy.is_shutdown():
        # 创建一个字典作为要发布的数据
        data = {
            'message': "Hello, world!"  # 设置消息的内容
        }

        # 将字典转换为JSON字符串
        json_data = json.dumps(data)

        # 创建一个String类型的消息
        message = String()
        message.data = json_data

        # 发布消息
        pub.publish(message)

        # 打印发布的消息
        rospy.loginfo("Published message: %s", message.data)

        # 按照设定的频率休眠
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass