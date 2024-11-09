#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import rospkg
import yaml
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

rospack = rospkg.RosPack()
package_path = rospack.get_path('robot_nav_pkg')
charging_file = os.path.join(package_path, 'config/task/charging.yaml')

def load_waypoints(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
        return data.get('waypoints', {})
    
def navigate_to(pose):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pose['position']['x']
    goal.target_pose.pose.position.y = pose['position']['y']
    goal.target_pose.pose.orientation.z = pose['orientation']['z']
    goal.target_pose.pose.orientation.w = pose['orientation']['w']

    rospy.loginfo("导航到路径点: %s", pose)
    client.send_goal(goal)
    client.wait_for_result()

def main():
    rospy.init_node('start_charging')
    # 读取文件，再执行任务
    waypoints = load_waypoints(charging_file)
    
    for waypoint_name, pose in waypoints.items():
        rospy.loginfo("到达充电点位: %s", waypoint_name)
        navigate_to(pose)
    rospy.loginfo("等待任务指令...")

if __name__ == '__main__':
    main()
