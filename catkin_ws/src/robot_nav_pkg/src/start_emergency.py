#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import rospkg
import yaml
import actionlib
from std_msgs.msg import String 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from robot_nav_pkg.srv import RobotTXData, RobotTXDataResponse
import threading  # 用于异步处理导航

class EmergencyNode:
    def __init__(self): 
        self.service = rospy.Service('/emergency_waypoint', RobotTXData, self.handle_emergency_point)
        self.command_publisher = rospy.Publisher('command_topic', String, queue_size=10)
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('robot_nav_pkg') 
        self.emergency_file = os.path.join(package_path,"config/task/emergency.yaml")
    
        self.emergency_waypoint_pub = rospy.Publisher('EMERGENCY_WAYPOINTS',MoveBaseGoal,queue_size=10)
   
    def navigate_to(self, pose):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pose['position']['x']
        goal.target_pose.pose.position.y = pose['position']['y']
        goal.target_pose.pose.orientation.z = pose['orientation']['z']
        goal.target_pose.pose.orientation.w = pose['orientation']['w']

        rospy.sleep(1)
        # 将要去往的点位保存发布给navFunc.py，实现恢复的时候可以继续导航至之前未到达的点位
        self.emergency_waypoint_pub.publish(goal)

        rospy.loginfo("导航到路径点: %s", pose)
        client.send_goal(goal)
        client.wait_for_result()
        rospy.loginfo("到达目标点")

    def load_waypoints(self, file_name):
        if os.path.exists(file_name):
            with open(file_name, 'r') as file:
                try:
                    data = yaml.safe_load(file)
                    waypoints = {}
                    if 'waypoints' in data:
                        # 进入waypoints字典
                        for key, value in data['waypoints'].items():
                            # 将 waypoint_x 按照 '_' 分割
                            if '_' in key:
                                parts = key.split('_')
                                if len(parts) == 2 and parts[1].isdigit():  # 确保后缀是数字
                                    waypoint_id = parts[1]  # 提取waypoint_x中的x作为字符串
                                    waypoints[waypoint_id] = value   # 存储waypoint_x对应的坐标信息
                                else:
                                    rospy.logwarn("无效的路径点键值: {key}")
                            else:
                                rospy.logwarn("键值不符合格式: {key}")
                    else:
                        rospy.logwarn("YAML文件中未找到'waypoints'键")
                    return waypoints
                except yaml.YAMLError as e:
                    rospy.logerr("加载文件时出错: %s", e)
        return None

    def handle_emergency_point(self, req):
        #读取yaml文件，提取出所有的waypoint中的x，可以是string类型
        waypoints = self.load_waypoints(self.emergency_file)
        # 判断 req 是否在 waypoints 中，如果在就去往这个 req 所指的坐标点
        if req.command in waypoints:
            # 如果 req 对应的 waypoint 存在，导航到该waypoint
            waypoint_pose = waypoints[req.command]
            rospy.loginfo("收到导航到 waypoint_{req.command} 的请求")
            # 向command_topic发布emergency命令
            self.command_publisher.publish("emergency")	
            # 导航与响应并行执行 ， 可以立刻收到响应
            #self.navigate_to(waypoint_pose)
            nav_thread = threading.Thread(target=self.navigate_to, args=(waypoint_pose,))
            nav_thread.start()
            return RobotTXDataResponse(success=True, message="执行emergency任务")
        else:
            rospy.logwarn("未找到 waypoint_{req.command}")
            return RobotTXDataResponse(success=False, message="无效的waypoint")

if __name__ == '__main__':
    rospy.init_node('start_emergency',anonymous=True)
    EmergencyNode()
    rospy.loginfo("Emergency service is ready")
    rospy.spin()

