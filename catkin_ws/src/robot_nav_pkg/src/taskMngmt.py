#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from robot_nav_pkg.srv import RobotTXData, RobotTXDataResponse
import os
import rospkg

def handle_task_management(req):
    # 获取ROS包路径
    rospack = rospkg.RosPack()
    # 构建指向config/task文件夹的绝对路径
    package_path = rospack.get_path('robot_nav_pkg')
    task_folder_path = os.path.join(package_path, 'config', 'task')
    
    command = req.command  # 使用正确的属性名获取命令
    response = RobotTXDataResponse()
    
    if command == "new_patrol":
        try:
            # 创建或覆盖patrolling.yaml文件
            file_path = os.path.join(task_folder_path, "patrolling.yaml")
            with open(file_path, "w") as f:
                f.write("# Patrolling configuration\n")
            response.success = True
            response.message = "Created patrolling.yaml at {}".format(file_path)
        except Exception as e:
            response.success = False
            response.message = str(e)
    
    elif command == "delete_patrol":
        # 检查patrolling.yaml文件是否存在并删除
        file_path = os.path.join(task_folder_path, "patrolling.yaml")
        if os.path.exists(file_path):
            try:
                os.remove(file_path)
                response.success = True
                response.message = "Deleted patrolling.yaml at {}".format(file_path)
            except Exception as e:
                response.success = False
                response.message = str(e)
        else:
            response.success = False
            response.message = "patrolling.yaml does not exist at {}".format(file_path)
    
    elif command == "new_emergency":
        try:
            # 创建或覆盖emergency.yaml文件
            file_path = os.path.join(task_folder_path, "emergency.yaml")
            with open(file_path, "w") as f:
                f.write("# Emergency configuration\n")
            response.success = True
            response.message = "Created emergency.yaml at {}".format(file_path)
        except Exception as e:
            response.success = False
            response.message = str(e)
    
    elif command == "delete_emergency":
        # 检查emergency.yaml文件是否存在并删除
        file_path = os.path.join(task_folder_path, "emergency.yaml")
        if os.path.exists(file_path):
            try:
                os.remove(file_path)
                response.success = True
                response.message = "Deleted emergency.yaml at {}".format(file_path)
            except Exception as e:
                response.success = False
                response.message = str(e)
        else:
            response.success = False
            response.message = "emergency.yaml does not exist at {}".format(file_path)
    
    else:
        response.success = False
        response.message = "Received unknown task: {}".format(command)

    return response

def task_manager_server():
    """
    初始化ROS节点并创建任务管理服务
    """
    rospy.init_node('taskMngmt')
    s = rospy.Service('taskMngmt', RobotTXData, handle_task_management)
    rospy.loginfo("Task manager service is ready")
    rospy.spin()

if __name__ == '__main__':
    try:
        task_manager_server()
    except rospy.ROSInterruptException:
        pass
