#!/usr/bin/env python3
#coding=utf-8
import rospy
def color_ros():
    rospy.logdebug('This is a debug message')
    rospy.loginfo('This is an info message')
    rospy.logwarn('This is a warning message')
    rospy.logerr('This is an error message')
    rospy.logfatal('This is a fatal message')
def prGreen(*args):
    print("\033[92m", end='')
    for i, arg in enumerate(args):
        print(arg, end=' ')
    print("\033[00m")

def prRed(*args):
    print("\033[91m", end='')
    for i, arg in enumerate(args):
        print(arg, end=' ')
    print("\033[00m")

def prYellow(*args):
    print("\033[93m", end='')
    for i, arg in enumerate(args):
        print(arg, end=' ')
    print("\033[00m")

def prBlue(*args):
    print("\033[94m", end='')
    for i, arg in enumerate(args):
        print(arg, end=' ')
    print("\033[00m")

def prPurple(*args):
    print("\033[95m", end='')
    for i, arg in enumerate(args):
        print(arg, end=' ')
    print("\033[00m")

def prCyan(*args):
    print("\033[96m", end='')
    for i, arg in enumerate(args):
        print(arg, end=' ')
    print("\033[00m")

def prOrange(*args):
    print("\033[33m", end='')
    for i, arg in enumerate(args):
        print(arg, end=' ')
    print("\033[00m")

def prPink(*args):
    print("\033[95m", end='')
    for i, arg in enumerate(args):
        print(arg, end=' ')
    print("\033[00m")