#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import rospkg
import yaml
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import re
from collections import OrderedDict
from robot_nav_pkg.srv import RobotTXData, RobotTXDataResponse
import threading

# 初始化变量
rospack = rospkg.RosPack()
package_path = rospack.get_path('robot_nav_pkg')
patrolling_file = os.path.join(package_path, 'config/task/patrolling.yaml')

paused = False
cancelled = False
paused_goal = None
condition = threading.Condition()
client = None  # 全局客户端变量

# 从YAML文件加载路径点
# python2 环境下需要自主去给字典排序，python3.7以上 safe_load 函数会自动排序
def load_waypoints(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
        waypoints = data.get('waypoints', {})
        # 按照 waypoint_x 中的数字顺序进行排序
        sorted_waypoints = OrderedDict(sorted(waypoints.items(), key=lambda item: int(re.search(r'(\d+)$', item[0]).group())))
        return sorted_waypoints


# 导航到指定路径点
def navigate_to(pose):
    global paused_goal
    paused_goal = pose  # 保存当前导航目标

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pose['position']['x']
    goal.target_pose.pose.position.y = pose['position']['y']
    goal.target_pose.pose.orientation.z = pose['orientation']['z']
    goal.target_pose.pose.orientation.w = pose['orientation']['w']

    rospy.loginfo("导航到路径点: %s", pose)
    client.send_goal(goal)

    # 等待结果，检查是否成功到达目标点
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("导航失败.")
    else:
        rospy.loginfo("到达目标点")


# 处理暂停、恢复和取消服务
def handle_control_service(req):
    global paused, paused_goal
    if req.command == 'pause':
        rospy.loginfo("收到暂停命令")
        with condition:
            paused = True
            client.cancel_all_goals()
            condition.notify_all()  # 通知主线程进入等待状态
        return RobotTXDataResponse(message="任务已暂停", success=True)

    elif req.command == 'resume':
        rospy.loginfo("收到恢复命令")
        with condition:
            if paused_goal:
                paused = False
                rospy.loginfo("恢复到任务点: %s", paused_goal)
                # 重新导航到暂停点
                navigate_to(paused_goal)
                client.wait_for_result()
                rospy.loginfo("导航已恢复...")
                paused_goal = None  # 清空,避免重复导航
                condition.notify_all()  # 通知主线程恢复
                return RobotTXDataResponse(message="任务已恢复",success=True)
            else:
                rospy.logwarn("没有保存的暂停点，无法恢复")
                return RobotTXDataResponse(message="没有保存的暂停点", success=False)
        return RobotTXDataResponse(message="任务已恢复", success=True)

    elif req.command == 'cancel':
        rospy.loginfo("收到取消命令")
        with condition:
            client.cancel_all_goals()
            cancelled = True
            condition.notify_all()
        return RobotTXDataResponse(message="所有巡检任务已取消", success=True)

    else:
        return RobotTXDataResponse(success=False, message="未知命令")

def main():
    global paused, cancelled, client
    rospy.init_node('start_patroll')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server() 

    rospy.Service('navFunc', RobotTXData, handle_control_service)
    waypoints = load_waypoints(patrolling_file)

    for waypoint_name, pose in waypoints.items():
        # 检查是否收到取消命令
        if cancelled:
            rospy.loginfo("巡检任务已取消，退出导航")
            break

        # 检查是否收到暂停命令
        with condition:
            while paused:
                rospy.loginfo("任务已暂停，等待恢复...")
                condition.wait()  # 主线程等待接收恢复信号

        rospy.loginfo("Navigating to waypoint: %s", waypoint_name)
        navigate_to(pose)
        rospy.sleep(2)  # 停留一段时间后继续下一个路径点


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
