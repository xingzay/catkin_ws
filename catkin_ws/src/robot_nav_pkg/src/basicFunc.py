#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from robot_nav_pkg.srv import RobotTXData, RobotTXDataResponse
import subprocess
import time

# 保存建图、定位和导航命令进程的引用
mapping_process = None
localization_process = None
navigation_process = None

# 定义当前状态
current_state = "idle"

def close_rviz():
    global close_rviz
    try:
        rospy.loginfo("开始关闭 RVIZ ...")
        close_rviz = subprocess.call('pkill rviz', shell=True)
        if close_rviz == 0:
            rospy.loginfo("关闭 RVIZ 成功...")
    except Exception as e:
        rospy.logerr("关闭 RVIZ 失败: %s", str(e))
def start_mapping():
    global mapping_process
    try:
        if mapping_process is None or mapping_process.poll() is not None:
            # 如果建图进程不存在或已经终止，则启动建图进程
            rospy.loginfo("启动 Cartographer SLAM 建图...")
            # mapping_process = subprocess.Popen("roslaunch cartographer_ros cartographer_mapping.launch", shell=True)
            mapping_process = subprocess.Popen("roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=cartographer use_sim_time:=true", shell=True)
        else:
            rospy.logwarn("SLAM 建图已经在运行.")
    except Exception as e:
        rospy.logerr("启动 SLAM 建图失败: %s", str(e))


def stop_mapping():
    global mapping_process
    try:
        if mapping_process is not None and mapping_process.poll() is None:
            # 如果建图进程正在运行，则终止建图进程
            rospy.loginfo("终止 SLAM 建图进程...")
            mapping_process.terminate()
            mapping_process.wait()
            rospy.loginfo("SLAM 建图进程终止.")
            mapping_process = None
        else:
            rospy.logwarn("SLAM 建图进程未运行.")
    except Exception as e:
        rospy.logerr("终止 SLAM 建图进程失败: %s", str(e))


def save_map():
    global mapping_process
    try:
        if mapping_process is not None and mapping_process.poll() is None:
            # 如果建图进程正在运行，则保存地图并终止建图进程
            rospy.loginfo("保存地图并停止 SLAM 建图...")
            # return_code = subprocess.call('/home/kc/finish_slam_2d.sh', shell=True)
            return_code = subprocess.call('/home/x/finish_slam_2d.sh', shell=True)
            if return_code == 0:
                rospy.loginfo("地图保存成功.")
                message = "地图保存成功."
                success = True
            else:
                rospy.logerr("地图保存失败, 返回码: %d", return_code)
                message = "地图保存失败."
                success = False
            stop_mapping()
        else:
            rospy.logwarn("SLAM 建图进程未运行.")
            message = "SLAM 建图进程未运行."
            success = False
    except Exception as e:
        rospy.logerr("地图保存失败: %s", str(e))
        message = "地图保存失败."
        success = False
    return message, success


def start_localization():
    global localization_process
    try:
        if localization_process is None or localization_process.poll() is not None:
            # 如果定位进程不存在或已经终止，则启动定位进程
    
            # localization_process = subprocess.Popen("roslaunch cartographer_ros cartographer_localization.launch",
            #                                         shell=True)
            localization_process = subprocess.Popen("roslaunch turtlebot3_slam tb3_cartographer_location.launch",
                                                     shell=True)
            rospy.loginfo("启动 Cartographer 纯定位...")
        else:
            rospy.logwarn("定位已经在运行.")
    except Exception as e:
        rospy.logerr("启动定位失败: %s", str(e))


def stop_localization():
    global localization_process
    try:
        if localization_process is not None and localization_process.poll() is None:
            # 如果定位进程正在运行，则终止定位进程
            rospy.loginfo("终止定位进程...")
            localization_process.terminate()
            localization_process.wait()
            rospy.loginfo("定位进程终止.")
            localization_process = None
        else:
            rospy.logwarn("定位进程未运行.")
    except Exception as e:
        rospy.logerr("终止定位进程失败: %s", str(e))


def start_navigation():
    global localization_process, navigation_process
    # 关闭rviz
    close_rviz()
    try:
        # 确保定位进程已启动
        if localization_process is None or localization_process.poll() is not None:
            rospy.loginfo("定位进程未运行，启动定位进程...")
            start_localization()
            # 等待1秒确保定位进程启动
            rospy.loginfo("等待1秒以确保定位进程启动...")
            time.sleep(1)

        if navigation_process is None or navigation_process.poll() is not None:
            # 如果导航进程不存在或已经终止，则启动导航进程
            rospy.loginfo("启动导航...")
            # navigation_process = subprocess.Popen("roslaunch robot_nav_pkg cartographer_localization.launch",
            #                                       shell=True)
            navigation_process = subprocess.Popen("roslaunch turtlebot3_navigation tb3_location_navigation.launch",
                                                  shell=True)
            rospy.loginfo("导航已启动...")
        else:
            rospy.logwarn("导航已经在运行.")
    except Exception as e:
        rospy.logerr("启动导航失败: %s", str(e))


def stop_navigation():
    global navigation_process
    try:
        if navigation_process is not None and navigation_process.poll() is None:
            # 如果导航进程正在运行，则终止导航进程
            rospy.loginfo("终止导航进程...")
            navigation_process.terminate()
            navigation_process.wait()
            rospy.loginfo("导航进程终止.")
            navigation_process = None
            # 终止定位进程
            stop_localization()
        else:
            rospy.logwarn("导航进程未运行.")
    except Exception as e:
        rospy.logerr("终止导航进程失败: %s", str(e))


def handle_command(req):
    global current_state, mapping_process, localization_process, navigation_process
    command = req.command

    if command == "mapping":
        current_state = "mapping"
        start_mapping()
        return RobotTXDataResponse("建图启动命令已处理", True)
    elif command == "mapping_kill":
        stop_mapping()
        current_state = "idle"
        return RobotTXDataResponse("建图停止命令已处理", True)
    elif command == "savemap":
        message, success = save_map()
        current_state = "idle"
        return RobotTXDataResponse(message, success)
    elif command == "navigation":
        if current_state == "mapping" and mapping_process is not None:
            # 如果当前状态为建图且建图进程正在运行，则终止建图进程
            rospy.loginfo("启动导航前终止建图进程.")
            stop_mapping()
        current_state = "navigation"
        start_navigation()
        return RobotTXDataResponse("导航启动命令已处理", True)
    elif command == "navigation_kill":
        stop_navigation()
        current_state = "idle"
        return RobotTXDataResponse("导航和定位停止命令已处理", True)
    else:
        return RobotTXDataResponse("接收到无效指令", False)


def listener():
    rospy.init_node('basicFunc', anonymous=True)
    rospy.Service('basicFunc', RobotTXData, handle_command)
    rospy.loginfo("服务启动成功，等待命令...")
    rospy.spin()


if __name__ == '__main__':
    listener()
