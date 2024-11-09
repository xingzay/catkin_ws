#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import dynamic_reconfigure.client
from robot_nav_pkg.srv import DynamicConfigure, DynamicConfigureResponse
from std_msgs.msg import Float64

class Configure(): 
    def __init__(self): 
        self.config_service = rospy.Service('dynamic_reconfigure', DynamicConfigure, self.dynamic_reconfigure_callback)

        self.max_vel_x_pub = rospy.Publisher('max_vel_x', Float64, queue_size=10)
        self.max_vel_x_backwards_pub = rospy.Publisher('max_vel_x_backwards', Float64, queue_size=10)
        self.max_vel_theta_pub = rospy.Publisher('max_vel_theta', Float64, queue_size=10)
        self.acc_lim_x_pub = rospy.Publisher('acc_lim_x', Float64, queue_size=10)
        self.acc_lim_theta_pub = rospy.Publisher('acc_lim_theta', Float64, queue_size=10)

        self.client = dynamic_reconfigure.client.Client("/move_base_node/TebLocalPlannerROS", timeout=5)

        # 创建定时器，设置为每0.5秒发布一次
        self.timer = rospy.Timer(rospy.Duration(0.5), self.publish_parameters)

    def dynamic_reconfigure_callback(self, req):
        try:
            # 更新实例中的参数值
            max_vel_x = req.max_vel_x   # 前向最大线速度
            max_vel_x_backwards = req.max_vel_x_backwards  # 后向最大线速度
            max_vel_theta = req.max_vel_theta  
            acc_lim_x = req.acc_lim_x
            acc_lim_theta = req.acc_lim_theta

            # 使用 dynamic_reconfigure 客户端设置参数
            config = {
                'max_vel_x': max_vel_x,
                'max_vel_x_backwards': max_vel_x_backwards,
                'max_vel_theta': max_vel_theta,
                'acc_lim_x': acc_lim_x,
                'acc_lim_theta': acc_lim_theta
            }

            # 更新参数
            self.client.update_configuration(config)

            return DynamicConfigureResponse(success=True, message="TEB parameters updated successfully.")

        except Exception as e:  
            # 处理错误并返回失败响应
            rospy.logerr("Failed to update TEB parameters: {}".format(str(e)))
            return DynamicConfigureResponse(success=False, message="Failed to update TEB parameters: {}".format(str(e)))       

    def publish_parameters(self, event):
        try:
            # 从 dynamic_reconfigure 客户端获取当前参数值
            config = self.client.get_configuration()

            # 发布当前参数值
            self.max_vel_x_pub.publish(Float64(config['max_vel_x']))
            self.max_vel_x_backwards_pub.publish(Float64(config['max_vel_x_backwards']))
            self.max_vel_theta_pub.publish(Float64(config['max_vel_theta']))
            self.acc_lim_x_pub.publish(Float64(config['acc_lim_x']))
            self.acc_lim_theta_pub.publish(Float64(config['acc_lim_theta']))
        except Exception as e:  
            rospy.logerr("Failed to get and publish parameters: {}".format(str(e)))

if __name__ == "__main__":
    try:
        rospy.init_node('robot_cmd_configure', anonymous=True)
        Configure()
        rospy.loginfo("Robot_Cmd_Configure Server is ready")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
