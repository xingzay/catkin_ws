#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import rospkg
import yaml
import re
from visualization_msgs.msg import Marker
import time

id_counter = 0
tasks = {}
last_modified_times = {}  # 保存每个文件的上次修改时间

# 定义不同任务对应的YAML文件路径
rospack = rospkg.RosPack()
package_path = rospack.get_path('robot_nav_pkg')
config_path = os.path.join(package_path, 'config', 'task')  # 指定config/task路径
filenames = {
    'patrolling': os.path.join(config_path, 'patrolling.yaml'),
    'emergency': os.path.join(config_path, 'emergency.yaml'),
    'charging': os.path.join(config_path, 'charging.yaml'),
    'initial': os.path.join(config_path, 'initial.yaml')
}

def publish_waypoints_as_markers():
    global tasks
    global id_counter  # 使用全局id_counter，确保ID唯一
    rospy.loginfo("开始发布 {len(tasks)} 个任务的marker")
    for task_name, waypoints in tasks.items():
        rospy.loginfo("发布 {task_name} 的 {len(waypoints)} 个路径点")
        if 'patrolling' in task_name:
            color = (1.0, 0.0, 0.0)  # 红色
        elif 'emergency' in task_name:
            color = (1.0, 1.0, 0.0)  # 黄色
        elif 'charging' in task_name:
            color = (0.0, 0.0, 1.0)  # 蓝色
        elif 'initial' in task_name:
            color = (0.0, 1.0, 0.0)  # 绿色
        else:
            color = (1.0, 1.0, 1.0)  # 默认颜色（白色）
        for waypoint_name, pose in waypoints.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = task_name
            marker.id = id_counter  # 确保每个Marker都有唯一ID
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = pose['position']['x']
            marker.pose.position.y = pose['position']['y']
            marker.pose.position.z = 0  # 假设路径点在地面
            marker.pose.orientation.x = pose['orientation'].get('x', 0)
            marker.pose.orientation.y = pose['orientation'].get('y', 0)
            marker.pose.orientation.z = pose['orientation']['z']
            marker.pose.orientation.w = pose['orientation']['w']
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.a = 1.0
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]

            marker.lifetime = rospy.Duration(0)  # 保证marker持久显示
            marker_pub.publish(marker)
            rospy.loginfo("发布了 {task_name} 的路径点 {waypoint_name}，ID: {id_counter}")
            id_counter += 1  # 每发布一个Marker，增加id_counter

def marker_publisher_timer(event):
    # 定期检查文件是否更新
    if check_files_updated():
        load_all_waypoints()
        publish_waypoints_as_markers()

def load_all_waypoints():
    global tasks
    tasks.clear()  # 清空原有的任务
    for task_name, filename in filenames.items():
        tasks[task_name] = load_waypoints(filename)

def load_waypoints(filename):
    if os.path.exists(filename):
        with open(filename, 'r') as file:
            try:
                data = yaml.safe_load(file)
                if data and 'waypoints' in data:
                    return data['waypoints']  # 直接返回保持顺序的路径点字典
            except yaml.YAMLError as e:
                rospy.logerr("加载文件出错: %s", e)
    return {}

def check_files_updated():
    """检查文件是否有修改,如果有则返回True"""
    global last_modified_times
    updated = False
    for task_name, filename in filenames.items():
        if os.path.exists(filename):
            last_modified_time = os.path.getmtime(filename)
            if task_name not in last_modified_times or last_modified_time > last_modified_times[task_name]:
                # rospy.loginfo(f"{filename} 文件已更新，重新加载.")
                last_modified_times[task_name] = last_modified_time
                updated = True
    return updated

def main():
    global marker_pub
    rospy.init_node('display_marker')
    load_all_waypoints()
    marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=10)   
    # 使用定时器每秒检查文件更新，并重新发布marker
    rospy.Timer(rospy.Duration(1), marker_publisher_timer)  # 每秒调用一次 marker_publisher_timer
    rospy.spin()

if __name__ == '__main__':
    main()
