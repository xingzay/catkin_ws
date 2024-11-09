#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import yaml
import tf
import os
import rospkg
from robot_nav_pkg.srv import RobotTXData, RobotTXDataResponse  # 导入自定义服务协议
from collections import OrderedDict

class SavePointNode:
    def __init__(self):
        # 初始化tf监听器
        self.tf_listener = tf.TransformListener()
        # 创建服务
        self.service = rospy.Service('/savePoint', RobotTXData, self.handle_save_point)
        # 获取当前ROS包路径
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('robot_nav_pkg')  # 替换为你的包名

        # 定义保存路径为config/task目录下
        self.patrolling_file = os.path.join(package_path, 'config/task/patrolling.yaml')
        self.emergency_file = os.path.join(package_path, 'config/task/emergency.yaml')
        self.charging_file = os.path.join(package_path, 'config/task/charging.yaml')
        self.initial_file = os.path.join(package_path, 'config/task/initial.yaml')

        # 定义任务类型和颜色映射
        self.all_waypoints = {
            'save_patrolling': [],
            'save_emergency': [],
            'save_charging': [],
            'save_initial': []
        }
    
    def load_existing_waypoints(self, filename):
        # 如果文件存在，加载并返回现有waypoints，同时更新waypoint_counter
        if os.path.exists(filename):
            with open(filename, 'r') as file:
                try:
                    data = yaml.safe_load(file)
                    if data and 'waypoints' in data:
                        waypoints = data['waypoints']
                        # 根据现有点位数量设置waypoint_counter
                        sorted_waypoints = dict(sorted(waypoints.items(), key=lambda item: int(item[0].split('_')[1])))
                        self.waypoint_counter = len(sorted_waypoints) + 1
                        return sorted_waypoints
                except yaml.YAMLError as e:
                    rospy.logerr("加载文件时出错: %s", e)
        # 如果文件不存在或没有有效数据，初始化waypoint_counter
        self.waypoint_counter = 1
        return {}

    def handle_save_point(self, req):
        try:
            # 获取机器人在地图中的位姿
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

            # 动态加载最新的waypoints并保存新的位姿
            if req.command == 'save_patrolling':
                waypoints = self.load_existing_waypoints(self.patrolling_file)
                self.save_waypoint(pose, self.patrolling_file, waypoints)
                self.all_waypoints['save_patrolling'].append(pose)
            elif req.command == 'save_emergency':
                waypoints = self.load_existing_waypoints(self.emergency_file)
                self.save_waypoint(pose, self.emergency_file, waypoints)
                self.all_waypoints['save_emergency'].append(pose)
            elif req.command == 'save_charging':
                self.save_waypoint(pose, self.charging_file, {}, overwrite=True)
                self.all_waypoints['save_charging'].append(pose)
            elif req.command == 'save_initial':
                self.save_waypoint(pose, self.initial_file, {}, overwrite=True)
                self.all_waypoints['save_initial'].append(pose)
            else:
                rospy.logwarn("未知的命令: %s", req.command)
                return RobotTXDataResponse(message="Unknown command.", success=False)

            return RobotTXDataResponse(message="{req.command.replace('save_', '').capitalize()} point saved.", success=True)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("获取机器人位置失败")
            return RobotTXDataResponse(message="Failed to get robot position.", success=False)
        
    def save_waypoint(self, pose, filename, waypoints, overwrite=False):
        if not overwrite:
            waypoint_name = 'waypoint_{}'.format(self.waypoint_counter)
            waypoints[waypoint_name] = pose
            self.waypoint_counter += 1
        else:
            waypoints = {'waypoint_1': pose}
            self.waypoint_counter = 2
        # 在保存之前，对waypoints进行排序
        self.save_waypoints_to_file(filename, waypoints)

    def save_waypoints_to_file(self, filename, waypoints):
        try:
            # 自定义 OrderedDumper 内部类
            class OrderedDumper(yaml.SafeDumper):
                def represent_dict(self, data):
                    return self.represent_mapping('tag:yaml.org,2002:map', data.items())

            OrderedDumper.add_representer(OrderedDict, OrderedDumper.represent_dict)
            
            sorted_waypoints = OrderedDict(sorted(waypoints.items(), key=lambda item: int(item[0].split('_')[1])))
            
            # 写入 YAML 文件
            with open(filename, 'w') as file:
                yaml.dump({'waypoints': sorted_waypoints}, file, Dumper=OrderedDumper, default_flow_style=False, allow_unicode=True)
                rospy.loginfo("位姿已保存到文件: %s", filename)
        except IOError as e:
            rospy.logerr("保存位姿到文件失败: %s", e)
            
if __name__ == '__main__':
    rospy.init_node('save_point_node')
    SavePointNode()
    rospy.loginfo("Save Point service is ready")
    rospy.spin()
