#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseActionGoal,MoveBaseAction,MoveBaseGoal
from robot_nav_pkg.srv import RobotTXData, RobotTXDataResponse

class NavigationControl:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('navFunc',anonymous=True)
        self.current_goal = None  # 当前目标位姿
        self.paused = False  # 导航是否暂停

        self.saved_goal = None

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.set_current_goal)
        self.emergency_waypoint_sub = rospy.Subscriber('EMERGENCY_WAYPOINTS',MoveBaseGoal,self.set_emergency_goal)
        # 服务服务器
        self.service = rospy.Service('navFunc', RobotTXData, self.handle_control_service)

    def handle_control_service(self, req):
        if req.command == 'pause':
            return self.pause_navigation()
        elif req.command == 'resume':
            return self.resume_navigation()
        elif req.command == 'cancel':
            return self.cancel_navigation()
        else:
            return RobotTXDataResponse("Unknown command.", False)

    def pause_navigation(self):
        if not self.paused and self.current_goal is not None:
            self.paused = True
            self.saved_goal = self.current_goal  # 保存当前目标
            self.move_base_client.cancel_all_goals()
            rospy.loginfo("Navigation paused.")
            return RobotTXDataResponse("Navigation paused.", True)
        elif self.paused:
            rospy.loginfo("Navigation is already paused.")
            return RobotTXDataResponse("Navigation is already paused.", False)
        else:
            rospy.loginfo("No active navigation to pause.")
            return RobotTXDataResponse("No active navigation to pause.", False)

    def resume_navigation(self):
        if self.paused and self.saved_goal is not None:
            self.paused = False
            
            # 创建新的 MoveBaseGoal 对象,send_goal 必须是 movebasegoal.target_pose 类型
            new_goal = MoveBaseGoal()
            new_goal.target_pose = self.saved_goal.goal.target_pose  # 从 saved_goal 提取目标位姿
            
            self.move_base_client.send_goal(new_goal)
            
            # 清空 saved_goal
            self.saved_goal = None
            rospy.loginfo("Navigation resumed.")
            return RobotTXDataResponse("Navigation resumed.", True)
        elif not self.paused:
            rospy.loginfo("Navigation is not paused.")
            return RobotTXDataResponse("Navigation is not paused.", False)
        else:
            rospy.loginfo("No active navigation to resume.")
            return RobotTXDataResponse("No active navigation to resume.", False)

    def cancel_navigation(self):
        if self.current_goal is not None:
            self.move_base_client.cancel_all_goals()
            self.current_goal = None
            self.paused = False
            self.saved_goal = None
            rospy.loginfo("Navigation cancelled.")
            return RobotTXDataResponse("Navigation cancelled.", True)
        else:
            rospy.loginfo("No active navigation to cancel.")
            return RobotTXDataResponse("No active navigation to cancel.", False)
        
    def set_current_goal(self,goal):
        self.current_goal = goal
        rospy.loginfo("Current goal set.")
        
    # 接收emergency将要去往的点位，并传给current_goal，防止current_goal为None
    def set_emergency_goal(self,goal):
        self.current_goal = goal
        rospy.loginfo("Emergency goal set.")

if __name__ == '__main__':
    try:
        nav_control = NavigationControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
