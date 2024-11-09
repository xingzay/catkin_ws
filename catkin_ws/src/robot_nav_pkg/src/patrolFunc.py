#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import yaml
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import os
import rospkg
import threading
from visualization_msgs.msg import Marker
import re
from robot_nav_pkg.srv import RobotTXData, RobotTXDataResponse  # 导入自定义服务定义
from collections import OrderedDict
# 管理当前任务线程的全局变量
current_task_thread = None
task_lock = threading.Lock()  # 线程锁，用于同步任务
stop_flag = False  # 用于停止任务的共享标志
pause_flag = False  # 新增：用于暂停任务的标志
paused_goal = None  # 新增：用于存储暂停时的目标

# 导航相关的全局变量
rospack = rospkg.RosPack()
package_path = rospack.get_path('robot_nav_pkg')  # 获取robot_nav_pkg功能包路径
config_path = os.path.join(package_path, 'config', 'task')  # 指定config/task路径

# 任务控制的条件变量
condition = threading.Condition()

# 定义不同任务对应的YAML文件路径
filenames = {
    'patrolling': os.path.join(config_path, 'patrolling.yaml'),
    'emergency': os.path.join(config_path, 'emergency.yaml'),
    'charging': os.path.join(config_path, 'charging.yaml')
}
tasks = {}
patrolling = False  # 标记是否在执行巡逻任务
previous_task = None  # 记录之前的任务

task_name = ["charging", "patrolling", "emergency", "cancel", "pause", "resume"]

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
    # python2 环境下需要自主去给字典排序，python3.7以上 safe_load 函数会自动排序
    if os.path.exists(filename):
        with open(filename, 'r') as file:
            try:
                data = yaml.safe_load(file)
                waypoints = data.get('waypoints', {})
                # 按照 waypoint_x 中的数字顺序进行排序
                sorted_waypoints = OrderedDict(sorted(waypoints.items(), key=lambda item: int(re.search(r'(\d+)$', item[0]).group())))
                return sorted_waypoints
            except yaml.YAMLError as e:
                rospy.logerr("加载文件出错: %s", e)
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
    global paused_goal  
    paused_goal = pose # 在导航时保存当前的目标
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
    global stop_flag, pause_flag, patrolling, previous_task
    waypoints = tasks.get(task_name, {})
    for waypoint_name, pose in waypoints.items():
        with condition:
            while pause_flag:  # 如果被暂停，等待恢复
                rospy.loginfo("任务已暂停")
                condition.wait()  # 等待恢复信号
        if stop_flag or (not patrolling and task_name == 'patrolling'):
            rospy.loginfo("%s 任务已停止", task_name.capitalize())
            return
        rospy.loginfo("Navigating to waypoint: %s", waypoint_name)
        navigate_to(pose)
        rospy.sleep(2)
    if once and previous_task:
        patrolling = True
        execute_patrolling()

def execute_patrolling():
    # 执行巡逻任务，循环执行直到停止
    global stop_flag, pause_flag, patrolling
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
    global current_task_thread, stop_flag, pause_flag, paused_goal, patrolling, previous_task
    load_all_waypoints()
    with task_lock:
        if req.command in task_name:
            if current_task_thread is not None and current_task_thread.is_alive():
                rospy.loginfo("正在停止当前任务")
                stop_flag = True
                client.cancel_all_goals()
                current_task_thread.join()
                stop_flag = False
                rospy.sleep(0.5)
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
        
def handle_navigate_request(req):
    global pause_flag, paused_goal
    with task_lock:
        if req.command in task_name:
            if req.command == 'pause':
                rospy.loginfo("收到暂停命令")
                with condition:
                    pause_flag = True
                    client.cancel_all_goals()  
                    return RobotTXDataResponse(message="任务已暂停", success=True)    
            #  navigate_to(paused_goal) 与 execute_task 线程冲突,所以产生跳点现象
            #  添加 condition = threading.Condition() 对象来同步线程,在执行 navigate_to(paused_goal) 函数时阻止 execute_task 函数的执行
            #  等待到达暂停之前的导航点后,再恢复 execute_task 线程
            elif req.command == 'resume':
                rospy.loginfo("收到恢复命令")
                if paused_goal:
                    with condition:
                        pause_flag = False
                        rospy.loginfo("恢复到任务点: %s", paused_goal)
                        navigate_to(paused_goal) 
                        client.wait_for_result()
                        condition.notify_all()  # 通知所有暂停的线程恢复
                        paused_goal = None  # 清空,避免重复导航
                    return RobotTXDataResponse(message="任务已恢复", success=True)
                else:
                    return RobotTXDataResponse(message="没有任务可以恢复", success=False)
        else:
            rospy.loginfo("无效命令，无任务执行")
            return RobotTXDataResponse(message="无效命令", success=False)
def listener():
    try:
        rospy.init_node('patrolFunc', anonymous=True)

        patrol_service = rospy.Service('/patrolFunc', RobotTXData, handle_task_request)
        rospy.loginfo("PatrolFunc service is ready...")

        nav_service = rospy.Service('/navFunc', RobotTXData, handle_navigate_request)
        rospy.loginfo("NavFunc service is ready...")
        
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
