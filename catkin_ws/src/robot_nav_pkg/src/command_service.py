#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from robot_nav_pkg.srv import RobotTXData, RobotTXDataResponse
from std_msgs.msg import String

# 创建发布者
command_pub = None
# 当前状态初始化为空
current_state = None

# 订阅状态机的状态更新回调函数
def state_update_callback(msg):
    global current_state
    # 更新当前状态
    if current_state != msg.data:  # 只在状态变化时更新
        current_state = msg.data
        rospy.loginfo("状态机当前状态: %s" % current_state)

# 服务回调函数，接收客户端请求
def handle_robot_command(req):
    global current_state
    rospy.loginfo("接收到服务请求: %s" % req.command)

    # 创建响应对象
    res = RobotTXDataResponse()

    # 如果状态还没有被初始化，返回错误
    if current_state is None:
        res.message = "状态机未初始化"
        res.success = False
        return res

    # 根据状态机当前状态决定有效命令
    if current_state == 'INITIALIZATION':
        valid_commands = ['mapping', 'navigation']
    elif current_state == 'IDLE':
        valid_commands = ['navigation_kill', 'patrolling', 'emergency', 'charging', 'initial', 'start_save_point', 'start_task']
    elif current_state == 'MAPPING':
        valid_commands = ['save_map']
    elif current_state == 'PATROLLING':
        valid_commands = ['emergency', 'charging', 'initial']
    elif current_state == 'CHARGING':
        valid_commands = ['initial', 'emergency']
    elif current_state == 'EMERGENCY':
        valid_commands = ['initial']
    else:
        valid_commands = []

    # 检查命令是否合法
    if req.command in valid_commands:
        # 发布命令到 command_topic 状态机
        command_pub.publish(req.command)
        res.message = "命令 '%s' 已发送" % req.command
        res.success = True
    else:
        res.message = "无效命令 '%s' 在当前状态 '%s'" % (req.command, current_state)
        res.success = False

    return res

# 服务节点函数
def command_service_server():
    global command_pub
    # 初始化 ROS 节点
    rospy.init_node('robot_command_service_server')

    # 创建服务，服务类型为 RobotTXData，回调函数为 handle_robot_command
    rospy.Service('robotCmd', RobotTXData, handle_robot_command)

    # 创建发布者，发布话题为 command_topic，消息类型为 String
    command_pub = rospy.Publisher('command_topic', String, queue_size=10)

    # 订阅状态机的状态更新话题
    rospy.Subscriber('state_topic', String, state_update_callback)

    rospy.loginfo("机器人命令服务已启动，等待状态机状态更新和客户端命令...")
    
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    command_service_server()

