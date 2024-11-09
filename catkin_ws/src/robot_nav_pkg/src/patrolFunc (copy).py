#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import yaml
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import os
import rospkg
from collections import OrderedDict
import threading
from visualization_msgs.msg import Marker
import re
from robot_nav_pkg.srv import RobotTXData, RobotTXDataResponse  # 导入自定义服务定义

# 管理当前任务线程的全局变量
current_task_thread = None
task_lock = threading.Lock()  # 线程锁，用于同步任务
stop_flag = False  # 用于停止任务的共享标志

# 导航相关的全局变量
rospack = rospkg.RosPack()
package_path = rospack.get_path('robot_nav_pkg')  # 获取robot_nav_pkg功能包路径
config_path = os.path.join(package_path, 'config', 'task')  # 指定config/task路径

# 定义不同任务对应的YAML文件路径
filenames = {
    'patrolling': os.path.join(config_path, 'patrolling.yaml'),
    'emergency': os.path.join(config_path, 'emergency.yaml'),
    'charging': os.path.join(config_path, 'charging.yaml')
}
tasks = {}
patrolling = False  # 标记是否在执行巡逻任务
previous_task = None  # 记录之前的任务

task_name = ["charging", "patrolling", "emergency", "cancel"]

# 发布可视化标记的Publisher
marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=10)

def load_all_waypoints():
    # 加载所有任务的路径点
    global tasks
    with task_lock:
        for task_name, filename in filenames.items():
            tasks[task_name] = load_waypoints(filename)

def load_waypoints(filename):
    # 从YAML文件加载路径点
    if os.path.exists(filename):
        with open(filename, 'r') as file:
            try:
                data = yaml.safe_load(file)
                if data and 'waypoints' in data:
                    # waypoints = OrderedDict(sorted(data['waypoints'].items()))
                    # return waypoints
                    return data['waypoints']  # 直接加载YAML文件zhong的路径点
            except yaml.YAMLError as e:
                rospy.logerr("加载文件出错: %s", e)
    # return OrderedDict()
    return {}

def publish_waypoints_as_markers():
    global tasks
    with task_lock:
        for task_name, waypoints in tasks.items():
            if 'patrolling' in task_name:
                color = (1.0, 0.0, 0.0)  # 红色
            elif 'emergency' in task_name:
                color = (1.0, 1.0, 0.0)  # 黄色
            elif 'charging' in task_name:
                color = (0.0, 0.0, 1.0)  # 蓝色
            else:
                color = (1.0, 1.0, 1.0)  # 默认颜色（白色）

            for waypoint_name, pose in waypoints.items():
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = task_name
                marker.id = int(re.sub("\D", "", waypoint_name))  # 从路径点名称提取ID
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = pose['position']['x']
                marker.pose.position.y = pose['position']['y']
                marker.pose.position.z = 0  # 假设路径点在地面
                marker.pose.orientation.x = pose['orientation'].get('x', 0)
                marker.pose.orientation.y = pose['orientation'].get('y', 0)
                marker.pose.orientation.z = pose['orientation']['z']
                marker.pose.orientation.w = pose['orientation']['w']
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]

                marker_pub.publish(marker)

def marker_publisher_timer(event):
    # 定期调用发布marker的函数
    publish_waypoints_as_markers()

def navigate_to(pose):
    # 导航至指定路径点
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pose['position']['x']
    goal.target_pose.pose.position.y = pose['position']['y']
    goal.target_pose.pose.orientation.z = pose['orientation']['z']
    goal.target_pose.pose.orientation.w = pose['orientation']['w']

    rospy.loginfo("正在导航到: %s", pose)
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    rospy.loginfo("已到达目标: %s", result)

def stop_robot():
    # 停止机器人在当前位置
    client.cancel_all_goals()  # 取消所有导航目标
    rospy.loginfo("所有巡检任务已取消，机器人停在原地")

def execute_task(task_name, once=False):
    # 执行指定任务
    global stop_flag, patrolling, previous_task
    waypoints = tasks.get(task_name, {})
    for waypoint_name, pose in waypoints.items():
        if stop_flag or (not patrolling and task_name == 'patrolling'):
            rospy.loginfo("%s 任务已停止", task_name.capitalize())
            return
        rospy.loginfo("Navigating to waypoint: %s", waypoint_name)  # 打印当前路径点名称
        navigate_to(pose)
        rospy.sleep(2)
    if once and previous_task:
        patrolling = True
        execute_patrolling()

def execute_patrolling():
    # 执行巡逻任务，循环执行直到停止
    global stop_flag, patrolling
    while not stop_flag and patrolling and not rospy.is_shutdown():
        if stop_flag:
            rospy.loginfo("巡逻任务已停止")
            return
        execute_task('patrolling')

class TaskThread(threading.Thread):
    # 用于执行任务的线程类
    def __init__(self, task_func):
        super(TaskThread, self).__init__()
        self.task_func = task_func

    def run(self):
        try:
            self.task_func()
        except Exception as e:
            rospy.logerr("任务线程出错: %s", e)

def handle_task_request(req):
    # 处理服务请求，根据请求的命令执行相应的任务
    global current_task_thread, stop_flag, patrolling, previous_task
    load_all_waypoints()
    with task_lock:
        if req.command in task_name:
            if current_task_thread is not None and current_task_thread.is_alive():
                rospy.loginfo("正在停止当前任务")
                stop_flag = True
                # 先取消当前导航，再进行emergency、charging线程任务
                client.cancel_all_goals()
                current_task_thread.join() 
                stop_flag = False
                rospy.sleep(0.5) # 等待 0.5s 响应

            command = req.command
            rospy.loginfo("收到命令: %s", command)
            if command == 'charging':
                patrolling = False
                current_task_thread = TaskThread(lambda: execute_task('charging'))
            elif command == 'emergency':
                if patrolling:
                    patrolling = False
                    previous_task = 'patrolling'
                current_task_thread = TaskThread(lambda: execute_task('emergency', once=True))
            elif command == 'patrolling':
                patrolling = True
            #    load_all_waypoints()  # 加载所有任务路径点
                rospy.Timer(rospy.Duration(1.0), marker_publisher_timer)  # 每秒发布一次marker
                current_task_thread = TaskThread(execute_patrolling)
            elif command == 'cancel':
                patrolling = False
                stop_robot()
                return RobotTXDataResponse(message="巡检任务已取消，机器人已停止", success=True)

            current_task_thread.start()
            return RobotTXDataResponse(message="任务启动成功", success=True)
        else:
            rospy.loginfo("无效命令，无任务执行")
            return RobotTXDataResponse(message="无效命令", success=False)

def listener():
    # 初始化ROS节点，加载路径点并启动服务
    try:
        rospy.init_node('patrolFunc', anonymous=True)
        service = rospy.Service('/patrolFunc', RobotTXData, handle_task_request)
        rospy.loginfo("Service /patrolFunc has been started")
        global listener
        listener = tf.TransformListener()
        global client
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("正在关闭")

if __name__ == '__main__':
    listener()
