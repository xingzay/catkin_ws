#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import yaml
import actionlib
import rospkg
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

rospkg = rospkg.RosPack()
package_path = rospkg.get_path('robot_nav_pkg')
initial_file = os.path.join(package_path, 'config/task/initial.yaml')

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
    rospy.loginfo("到达目标点")

def main():
    rospy.init_node('initial_script')
    waypoints = load_waypoints(initial_file)
    for waypoint_name, pose in waypoints.items():
        rospy.loginfo("Navigating to waypoint: %s", waypoint_name)
        navigate_to(pose)
        rospy.sleep(2)  # 等待 2 秒

if __name__ == '__main__':
    main()
