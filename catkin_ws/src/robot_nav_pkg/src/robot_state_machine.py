#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import subprocess
from std_msgs.msg import String
import os
import signal

def terminate_process(name):
    """终止指定名称的进程"""
    try:
        process_list = subprocess.check_output(['ps', 'aux']).decode().splitlines()
        for process in process_list:
            if name in process and 'grep' not in process:
                pid = int(process.split()[1])
                os.kill(pid, signal.SIGTERM)
                rospy.loginfo("Terminated process: {name} with PID {pid}")
    except subprocess.CalledProcessError as e:
        rospy.logwarn("Error terminating process {name}: {e}")

class INITIALIZATION(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MAPPING', 'IDLE', 'INITIALIZATION'])
        self.last_command = None
        self.state_pub = rospy.Publisher('state_topic', String, queue_size=10)
        rospy.Subscriber('command_topic', String, self.command_callback)

    def command_callback(self, msg):
        rospy.loginfo("Received command in INITIALIZATION: {msg.data}")
        if msg.data:
            self.last_command = msg.data

    def execute(self, userdata):
        rospy.loginfo("Executing INITIALIZATION state")
        self.state_pub.publish('INITIALIZATION')  # 发布当前状态
        if self.last_command == 'mapping':
            self.last_command = None
            return 'MAPPING'
        elif self.last_command == 'navigation':
            self.last_command = None
            return 'IDLE'
        return 'INITIALIZATION'

class MAPPING(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['INITIALIZATION', 'MAPPING'])
        self.last_command = None
        self.mapping_process = None
        self.state_pub = rospy.Publisher('state_topic', String, queue_size=10)
        rospy.Subscriber('command_topic', String, self.command_callback)

    def command_callback(self, msg):
        rospy.loginfo("Received command in MAPPING: {msg.data}")
        if msg.data:
            self.last_command = msg.data

    def execute(self, userdata):
        rospy.loginfo("Executing MAPPING state")
        self.state_pub.publish('MAPPING')  # 发布当前状态
        if self.mapping_process is None:
            # 启动建图进程
            self.mapping_process = subprocess.Popen(
                ['roslaunch', 'cartographer_ros', 'cartographer_mapping.launch',]
            )
            rospy.loginfo("Mapping process started.")

        while self.mapping_process.poll() is None:
            if self.last_command == 'save_map':
                self.last_command = None
                rospy.loginfo("Saving map and terminating mapping process.")
                save_map_process = subprocess.Popen(
                    ['bash', '-c', 'cd ~/cartographer_ws && ./finish_slam_2d.sh']
                )
                save_map_process.wait()
                if self.mapping_process:
                    self.mapping_process.terminate()
                    rospy.loginfo("Mapping process terminated.")
                    self.mapping_process = None
                terminate_process('rviz')
                return 'INITIALIZATION'
            
            if self.last_command == 'initial':
                self.last_command = None
                if self.mapping_process:
                    self.mapping_process.terminate()
                    rospy.loginfo("Mapping process terminated.")
                    self.mapping_process = None
                subprocess.Popen(['rosrun', 'robot_nav_pkg', 'start_initial.py'])
                return 'IDLE'
            
            rospy.sleep(1)

        return 'MAPPING'

class IDLE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['INITIALIZATION', 'IDLE', 'PATROLLING', 'EMERGENCY', 'CHARGING'])
        self.last_command = None
        self.navigation_process = None
        self.localzation_process = None

        self.display_process = None
        self.dynamic_config_process = None
        self.state_pub = rospy.Publisher('state_topic', String, queue_size=10)
        subprocess.Popen(['rosrun', 'robot_nav_pkg', 'navFunc.py'])
        subprocess.Popen(['rosrun', 'robot_nav_pkg', 'taskMngmt.py'])
        subprocess.Popen(['rosrun', 'robot_nav_pkg', 'savePoint.py'])
        emergency_process = subprocess.Popen(['rosrun', 'robot_nav_pkg', 'start_emergency.py'])
        rospy.Subscriber('command_topic', String, self.command_callback)

    def command_callback(self, msg):
        rospy.loginfo("Received command in IDLE: {msg.data}")
        if msg.data:
            self.last_command = msg.data

    def execute(self, userdata):
        rospy.loginfo("Executing IDLE state")
        self.state_pub.publish('IDLE')  # 发布当前状态
        if self.last_command == 'navigation':
            self.last_command = None  # 清除命令
            if self.navigation_process is None:
                # 启动导航进程
                self.localzation_process = subprocess.Popen(['roslaunch', 'cartographer_ros', 'cartographer_localization.launch'])
                rospy.sleep(1)
                self.navigation_process = subprocess.Popen(['roslaunch', 'robot_nav_pkg', 'cartographer_localization.launch'])
                rospy.sleep(0.5)
                # 显示任务点
                self.display_process = subprocess.Popen(['roslaunch','robot_nav_pkg','display.launch'])
                rospy.sleep(0.5)
                # 动态调参
                self.dynamic_config_process = subprocess.Popen(['rosrun','robot_nav_pkg','dynamic_configure.py'])
                rospy.loginfo("Navigation process started.")
            return 'IDLE'

        if self.last_command == 'navigation_kill':
            self.last_command = None  # 清除命令
            if self.navigation_process:
                self.localzation_process.terminate()
                self.navigation_process.terminate()  # 终止导航进程
                rospy.loginfo("Navigation process terminated.")
                self.navigation_process = None
            self.localzation_process = None
            terminate_process('rviz')  # 终止RViz进程
            return 'INITIALIZATION'

        elif self.last_command == 'patrolling':
            self.last_command = None  # 清除命令
            return 'PATROLLING'

        elif self.last_command == 'emergency':
            self.last_command = None  # 清除命令
            return 'EMERGENCY'

        elif self.last_command == 'charging':
            self.last_command = None  # 清除命令
            return 'CHARGING'

        elif self.last_command == 'initial':
            self.last_command = None  # 清除命令
            # 启动返回初始点脚本，保持状态不变
            subprocess.Popen(['rosrun', 'robot_nav_pkg', 'start_initial.py'])
            rospy.loginfo("Initial process started.")
            return 'IDLE'

        return 'IDLE'

class PATROLLING(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['IDLE', 'EMERGENCY', 'CHARGING'])
        self.last_command = None
        self.patrolling_process = None
        self.state_pub = rospy.Publisher('state_topic', String, queue_size=10)
        rospy.Subscriber('command_topic', String, self.command_callback)

    def command_callback(self, msg):
        rospy.loginfo("Received command in PATROLLING: {msg.data}")
        if msg.data:
            self.last_command = msg.data

    def execute(self, userdata):
        rospy.loginfo("Executing PATROLLING state")
        self.state_pub.publish('PATROLLING')
        if self.patrolling_process is None:
            self.patrolling_process = subprocess.Popen(['rosrun', 'robot_nav_pkg', 'start_patrolling.py'])
            rospy.loginfo("Patrolling process started.")
        
        while self.patrolling_process.poll() is None:
            if self.last_command == 'initial':
                self.last_command = None
                if self.patrolling_process:
                    self.patrolling_process.terminate()
                    rospy.loginfo("Patrolling process terminated.")
                    self.patrolling_process = None
                subprocess.Popen(['rosrun', 'robot_nav_pkg', 'start_initial.py'])
                 
                subprocess.Popen(['rosrun', 'robot_nav_pkg', 'navFunc.py'])  # 加上，实现去往初始点的时候暂停
                
                return 'IDLE'
            
            elif self.last_command == 'emergency':  # 处理 emergency 命令
                self.last_command = None
                if self.patrolling_process:
                    self.patrolling_process.terminate()
                    rospy.loginfo("Patrolling process terminated.")
                    self.patrolling_process = None
                return 'EMERGENCY'
            
            elif self.last_command == 'charging':  # 处理 charging 命令
                self.last_command = None
                if self.patrolling_process:
                    self.patrolling_process.terminate()
                    rospy.loginfo("Patrolling process terminated.")
                    self.patrolling_process = None
                return 'CHARGING'
            
            rospy.sleep(1)

        self.patrolling_process = None
        return 'IDLE'

class EMERGENCY(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['IDLE', 'EMERGENCY'])
        self.last_command = None
        self.emergency_process = None
        self.state_pub = rospy.Publisher('state_topic', String, queue_size=10)
        rospy.Subscriber('command_topic', String, self.command_callback)

    def command_callback(self, msg):
        rospy.loginfo("Received command in EMERGENCY: {msg.data}")
        if msg.data:
            self.last_command = msg.data

    def execute(self, userdata):
        rospy.loginfo("Executing EMERGENCY state")
        self.state_pub.publish('EMERGENCY')
        
        if self.emergency_process is None:
            # 重启之后，current_goal 重置为None,故无法暂停 ?
            # 原因已找到：因为 EMERGENCY 状态发布两次导致 navFunc 与 start_emergency 节点重复启动造成
            # 将 start_emergency 与 navFunc 节点 设为唯一性
            subprocess.Popen(['rosrun', 'robot_nav_pkg', 'navFunc.py']) 
            rospy.sleep(0.5)
            self.emergency_process = subprocess.Popen(['rosrun', 'robot_nav_pkg', 'start_emergency.py'])
            rospy.loginfo("Emergency process started.")
        
        while self.emergency_process.poll() is None:
            if self.last_command == 'initial':
                self.last_command = None
                if self.emergency_process:
                    self.emergency_process.terminate()
                    rospy.loginfo("Emergency process terminated.")
                    self.emergency_process = None
                subprocess.Popen(['rosrun', 'robot_nav_pkg', 'start_initial.py'])
                emergency_process = subprocess.Popen(['rosrun', 'robot_nav_pkg', 'start_emergency.py'])
                return 'IDLE'
            
            rospy.sleep(1)

        self.emergency_process = None
        return 'EMERGENCY'
    
class CHARGING(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['IDLE'])
        self.last_command = None
        self.charging_process = None
        self.state_pub = rospy.Publisher('state_topic', String, queue_size=10)
        rospy.Subscriber('command_topic', String, self.command_callback)

    def command_callback(self, msg):
        rospy.loginfo("Received command in CHARGING: {msg.data}")
        if msg.data:
            self.last_command = msg.data

    def execute(self, userdata):
        rospy.loginfo("Executing CHARGING state")
        self.state_pub.publish('CHARGING')
        if self.charging_process is None:
            self.charging_process = subprocess.Popen(['rosrun', 'robot_nav_pkg', 'start_charging.py'])
            rospy.loginfo("Charging process started.")
        
        while self.charging_process.poll() is None:
            if self.last_command == 'initial':
                self.last_command = None
                if self.charging_process:
                    self.charging_process.terminate()
                    rospy.loginfo("Charging process terminated.")
                    self.charging_process = None
                subprocess.Popen(['rosrun', 'robot_nav_pkg', 'start_initial.py'])
                return 'IDLE'
            
            elif self.last_command == 'emergency':
                self.last_command = None
                if self.charging_process:
                    self.charging_process.terminate()
                    rospy.loginfo("Charging process terminated.")
                    self.charging_process = None
                return 'EMERGENCY'
            
            rospy.sleep(1)

        self.charging_process = None
        return 'IDLE'
    
def main():
    rospy.init_node('robot_state_machine')

    # 创建状态机
    sm = smach.StateMachine(outcomes=['outcome4'])

    with sm:
        smach.StateMachine.add('INITIALIZATION', INITIALIZATION(), transitions={'MAPPING':'MAPPING', 'IDLE':'IDLE', 'INITIALIZATION':'INITIALIZATION'})
        smach.StateMachine.add('MAPPING', MAPPING(), transitions={'INITIALIZATION':'INITIALIZATION', 'MAPPING':'MAPPING'})
        smach.StateMachine.add('IDLE', IDLE(), transitions={'INITIALIZATION':'INITIALIZATION', 'IDLE':'IDLE', 'PATROLLING':'PATROLLING', 'EMERGENCY':'EMERGENCY', 'CHARGING':'CHARGING'})
        smach.StateMachine.add('PATROLLING', PATROLLING(), transitions={'IDLE':'IDLE', 'EMERGENCY':'EMERGENCY', 'CHARGING':'CHARGING'})
        smach.StateMachine.add('CHARGING', CHARGING(), transitions={'IDLE':'IDLE'})
        smach.StateMachine.add('EMERGENCY', EMERGENCY(), transitions={'IDLE':'IDLE', 'EMERGENCY':'EMERGENCY'})
        
    # 执行状态机
    outcome = sm.execute()

if __name__ == '__main__':
    main()
