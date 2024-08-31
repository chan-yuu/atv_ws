#!/usr/bin/env python3
#coding=utf-8
import rospy
import time
def color_ros():
    rospy.logdebug('This is a debug message')
    rospy.loginfo('This is an info message')
    rospy.logwarn('This is a warning message')
    rospy.logerr('This is an error message')
    rospy.logfatal('This is a fatal message')
def prGreen(*args):
    print("\033[32m", end='')
    for i, arg in enumerate(args):
        print(arg, end=' ')
    print("\033[32m")

def prRed(*args):
    print("\033[31m", end='')
    for i, arg in enumerate(args):
        print(arg, end=' ')
    print("\033[31m")

def prYellow(*args):
    print("\033[33m", end='')
    for i, arg in enumerate(args):
        print(arg, end=' ')
    print("\033[33m")

def prBlue(*args):
    print("\033[34m", end='')
    for i, arg in enumerate(args):
        print(arg, end=' ')
    print("\033[34m")

def prPurple(*args):
    print("\033[35m", end='')
    for i, arg in enumerate(args):
        print(arg, end=' ')
    print("\033[35m")

def prCyan(*args):
    print("\033[36m", end='')
    for i, arg in enumerate(args):
        print(arg, end=' ')
    print("\033[36m")

# for test:
# while not rospy.is_shutdown():
#     default = 129
#     prGreen("hello world", default)
#     time.sleep(1)