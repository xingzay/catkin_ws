#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import yaml
import tf
import os
import rospkg
from robot_nav_pkg.srv import RobotTXData, RobotTXDataResponse  # 导入自定义服务协议

class SavePointNode:
    def __init__(self):
        # 初始化 tf 监听器
        self.tf_listener = tf.TransformListener()
        # 创建服务
        self.service = rospy.Service('/savePoint', RobotTXData, self.handle_save_point)

        # 获取当前 ROS 包的路径
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('robot_nav_pkg')  # 替换为你的包名

        # 定义文件保存路径
        self.patrolling_file = os.path.join(package_path, 'config/task/patrolling.yaml')
        self.emergency_file = os.path.join(package_path, 'config/task/emergency.yaml')
        self.charging_file = os.path.join(package_path, 'config/task/charging.yaml')

        # 从 YAML 文件中加载现有的 waypoints
        self.patrolling_waypoints = self.load_existing_waypoints(self.patrolling_file)
        self.emergency_waypoints = self.load_existing_waypoints(self.emergency_file)
        self.patrolling_counter = self.get_max_waypoint_number(self.patrolling_waypoints) + 1
        self.emergency_counter = self.get_max_waypoint_number(self.emergency_waypoints) + 1

    def load_existing_waypoints(self, filename):
        """加载现有的 waypoints"""
        if os.path.exists(filename):
            try:
                with open(filename, 'r') as file:
                    data = yaml.load(file, Loader=yaml.SafeLoader)  # 使用 yaml.SafeLoader 替代 safe_load
                    if data and 'waypoints' in data:
                        return data['waypoints']
            except yaml.YAMLError as e:
                rospy.logerr("加载文件时出错: %s" % e)
        return {}

    def get_max_waypoint_number(self, waypoints):
        """获取最大 waypoint 编号"""
        if waypoints:
            numbers = [int(name.split('__')[-1]) for name in waypoints.keys() if name.startswith('waypoint__')]
            if numbers:
                return max(numbers)
        return 0

    def handle_save_point(self, req):
        """处理保存位置的请求"""
        try:
            # 获取当前机器人位置
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            pose = {
                'position': {
                    'x': trans[0],
                    'y': trans[1],
                    'z': trans[2]
                },
                'orientation': {
                    'x': rot[0],
                    'y': rot[1],
                    'z': rot[2],
                    'w': rot[3]
                }
            }
            # 根据请求的命令保存位置
            if req.command == 'save_patrolling':
                self.save_waypoint(pose, self.patrolling_file, self.patrolling_waypoints, 'patrolling')
                return RobotTXDataResponse(message="Patrolling point saved.", success=True)
            elif req.command == 'save_emergency':
                self.save_waypoint(pose, self.emergency_file, self.emergency_waypoints, 'emergency')
                return RobotTXDataResponse(message="Emergency point saved.", success=True)
            elif req.command == 'save_charging':
                self.save_waypoint(pose, self.charging_file, {}, 'charging', overwrite=True)
                return RobotTXDataResponse(message="Charging point saved.", success=True)
            else:
                rospy.logwarn("未知的命令: %s" % req.command)
                return RobotTXDataResponse(message="Unknown command.", success=False)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("获取机器人位置失败")
            return RobotTXDataResponse(message="Failed to get robot position.", success=False)

    def save_waypoint(self, pose, filename, waypoints, waypoint_type, overwrite=False):
        """保存位置信息到指定文件"""
        if not overwrite:
            if waypoint_type == 'patrolling':
                waypoint_name = 'waypoint__%d' % self.patrolling_counter
                self.patrolling_counter += 1
            elif waypoint_type == 'emergency':
                waypoint_name = 'waypoint__%d' % self.emergency_counter
                self.emergency_counter += 1
            waypoints[waypoint_name] = pose
        else:
            waypoint_name = 'waypoint__1'
            waypoints = {waypoint_name: pose}
            if waypoint_type == 'charging':
                self.patrolling_counter = 2
                self.emergency_counter = 2

        self.save_waypoints_to_file(filename, waypoints)

    def save_waypoints_to_file(self, filename, waypoints):
        """将 waypoints 保存到文件"""
        try:
            with open(filename, 'w') as file:
                yaml.dump({'waypoints': waypoints}, file, default_flow_style=False, allow_unicode=True)
                rospy.loginfo("位置信息已保存到文件: %s" % filename)
        except IOError as e:
            rospy.logerr("保存位置信息到文件失败: %s" % e)

if __name__ == '__main__':
    rospy.init_node('savePoint')
    SavePointNode()
    rospy.spin()
