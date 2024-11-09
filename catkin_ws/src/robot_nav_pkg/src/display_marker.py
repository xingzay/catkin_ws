#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import rospkg
import yaml
from visualization_msgs.msg import Marker
import time

tasks = {}
last_modified_times = {}  # 保存每个文件的上次修改时间
active_markers = {}  # 记录每个任务的 marker 信息

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
    global active_markers
    # rospy.loginfo(f"开始发布 {len(tasks)} 个任务的marker")

    # 发布现有任务的 marker
    for task_name, waypoints in tasks.items():
        # rospy.loginfo(f"发布 {task_name} 的 {len(waypoints)} 个路径点")
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

        # 初始化或重置任务的 marker 列表
        active_markers[task_name] = {}

        # 遍历路径点，使用索引作为 Marker ID
        for idx, (waypoint_name, pose) in enumerate(waypoints.items()):
            marker_id = idx  # 使用索引作为 Marker ID
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = task_name
            marker.id = marker_id  # 确保每个Marker都有唯一ID
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = pose['position']['x']
            marker.pose.position.y = pose['position']['y']
            marker.pose.position.z = 0  # 假设路径点在地面
            marker.pose.orientation.x = pose['orientation'].get('x', 0)
            marker.pose.orientation.y = pose['orientation'].get('y', 0)
            marker.pose.orientation.z = pose['orientation'].get('z', 0)
            marker.pose.orientation.w = pose['orientation'].get('w', 1)
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.a = 1.0
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]

            marker.lifetime = rospy.Duration(0)  # 保证marker持久显示
            marker_pub.publish(marker)
            # rospy.loginfo(f"发布了 {task_name} 的路径点 {waypoint_name}，ID: {marker_id}")
            active_markers[task_name][marker_id] = waypoint_name  # 记录 marker 信息

def clear_markers_for_task(task_name):
    """清除指定任务的所有 marker"""
    global active_markers
    if task_name in active_markers:
        # rospy.loginfo(f"清除 {task_name} 的所有 markers")
        for marker_id in active_markers[task_name]:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = task_name
            marker.id = marker_id
            marker.action = Marker.DELETE  # 删除 marker
            marker_pub.publish(marker)
        # 清除完后从 active_markers 中移除
        active_markers.pop(task_name)

def marker_publisher_timer(event):
    # 定期检查文件是否更新或被删除
    if check_files_updated():
        load_all_waypoints()
        publish_waypoints_as_markers()

def load_all_waypoints():
    global tasks
    tasks.clear()  # 清空任务内容
    # 只加载当前存在的文件
    for task_name, filename in filenames.items():
        if os.path.exists(filename):
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
    """检查文件是否有修改或删除,如果有则返回True"""
    global last_modified_times
    updated = False
    for task_name, filename in filenames.items():
        if os.path.exists(filename):
            last_modified_time = os.path.getmtime(filename)
            # 文件有变动，重新加载
            if task_name not in last_modified_times or last_modified_time > last_modified_times[task_name]:
                # rospy.loginfo(f"{filename} 文件已更新或新建，重新加载.")
                last_modified_times[task_name] = last_modified_time
                updated = True
        else:
            # 文件不存在，删除其 markers
            if task_name in last_modified_times:
                # rospy.loginfo(f"{filename} 文件已删除，清除任务点")
                clear_markers_for_task(task_name)
                last_modified_times.pop(task_name)  # 删除记录，等待重新创建时更新
                updated = True
    return updated

def main():
    global marker_pub
    rospy.init_node('display_marker')
    load_all_waypoints()
    marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=10)
    # 使用定时器每秒检查文件更新，并重新发布 marker
    rospy.Timer(rospy.Duration(1), marker_publisher_timer)
    rospy.loginfo("display_marker_node is already...")
    rospy.spin()

if __name__ == '__main__':
    main()

