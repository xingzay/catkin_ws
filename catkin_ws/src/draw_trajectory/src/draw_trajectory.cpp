#include<ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
ros::Publisher path_pub;
ros::Subscriber sub_point;
nav_msgs::Path myPath;
void pointHandler(const geometry_msgs::PoseStampedConstPtr& msg)
{
    ROS_INFO("Receive a Pose!!!!!");
    myPath.header.frame_id = "map";
    geometry_msgs::PoseStamped tempPose;
    tempPose.header.frame_id = "map";
    tempPose.header.stamp = ros::Time::now();
    tempPose.pose.position.x = msg->pose.position.x;
    tempPose.pose.position.y = msg->pose.position.y;
    tempPose.pose.position.z = msg->pose.position.z;
    myPath.poses.push_back(tempPose);
    path_pub.publish(myPath);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "drawTrajectory");
    ros::NodeHandle nh;

    
    sub_point = nh.subscribe("/move_base_simple/goal",10,&pointHandler);
    path_pub = nh.advertise<nav_msgs::Path>("myPath",10);
    while(ros::ok())
    {
        ros::spinOnce();
    }
}