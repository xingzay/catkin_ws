#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import threading

threads = []  
# 使用 Event 事件来同步定位完成后启动导航
localization_done = threading.Event()

localization_process = None
navigation_process = None
def close_rviz():
    rospy.loginfo("开始关闭RVIZ ...")
    if(os.system("pkill rviz") == 0):
        rospy.loginfo("关闭 RVIZ 成功")
    else:
        rospy.logwarn("关闭 RVIZ 失败")

def start_localization():
    global localization_process
    try:
        rospy.loginfo("开启 Cartographer 纯定位 ...")
        localization_process = os.system("roslaunch cartographer_ros cartographer_localization.launch")
        if localization_process is not None:
            rospy.loginfo("成功开启定位 ... ")
        else:
            rospy.logwarn("定位已经启动 ... ")
            
    except Exception as e:
        rospy.logwarn("启动定位失败:%s",e)

def start_navigation():
    global localization_process,navigation_process
    try:
        rospy.loginfo("开始启动导航...")
        navigation_process = os.system("roslaunch robot_nav_pkg cartographer_localization.launch")
        if localization_process is not None and navigation_process is not None:
            rospy.loginfo("导航启动成功...")
        else:
            rospy.loginfo("导航已经在运行。")
    except Exception as e:
        rospy.logerr("启动导航失败: %s", str(e))


if __name__ == '__main__':
    rospy.init_node('navigation')
    thread1 = threading.Thread(target = start_localization)
    thread2 = threading.Thread(target = start_navigation)
    # 启动线程
    thread1.start()
    thread2.start()
    thread1.join()
    thread2.join()
