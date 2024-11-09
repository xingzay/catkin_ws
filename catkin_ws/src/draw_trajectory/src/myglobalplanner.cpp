/*
 * navfn_ros_ext.cpp
 *
 *  Created on: Nov 28, 2016
 *      Author: thobotics
 */

#include "myglobalplanner.h"
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <algorithm>
#include <string>
#include <iostream>
#include <fstream>
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(navfn::myGlobalPlanner, nav_core::BaseGlobalPlanner);
namespace navfn {

	static const double PI= 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;

	myGlobalPlanner::myGlobalPlanner() :
		global_planner::GlobalPlanner(),initialized_(false),firstFlag_(true){}
	myGlobalPlanner::myGlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap_ros, std::string frame_id) :
		global_planner::GlobalPlanner(name, costmap_ros, frame_id){}


	// static const double PI= 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;

	// myGlobalPlanner::myGlobalPlanner():
	// 	global_planner::GlobalPlanner(), firstFlag_(true), initialized_(false){
  //   }
	// myGlobalPlanner::myGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros, std::string frame_id) :
	// 	global_planner::GlobalPlanner(name, costmap_ros->getCostmap(), frame_id){
  //   initialize(name,costmap_ros);
  //   }

	// void myGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  //   if(!initialized_)
	// 	{
  //     global_planner::GlobalPlanner::initialize(name, costmap_ros);
	// 	  ROS_INFO("Initialized NavfnROSExt");
  //     ros::NodeHandle nh;
  //     ros::NodeHandle pnh("~"+name);

  //     waypoint_sub = nh.subscribe("/move_base_simple/goal",10,&myGlobalPlanner::waypointsHandler, this);
  //     tempPath_pub = nh.advertise<nav_msgs::Path>("temp_path", 10);
  //   }
  //   else{return ;}
  
  // }
	void myGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		global_planner::GlobalPlanner::initialize(name, costmap_ros);
		ROS_INFO("Initialized NavfnROSExt");
    nh.param<float>("/myGlobalPlanner/markerScale",markerScale,1.0);
    nh.param<std::string>("/myGlobalPlanner/globalFrameID",global_frame_id,"map");
    mypath.header.frame_id = "map";
    waypointID = 0;
    tempMarker.header.frame_id = "map";
    tempMarker.type = 2;
    tempMarker.color.a = 1.0;
    tempMarker.color.r = 1.0;
    tempMarker.scale.x = markerScale;
    tempMarker.scale.y = markerScale;
    tempMarker.scale.z = markerScale;
    // waypoint_sub = mynh.subscribe("/move_base_simple/goal",10,&myGlobalPlanner::waypointsHandler, this);
    tempPath_pub = nh.advertise<nav_msgs::Path>("temp_path", 10);
    waypoints_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoints",10);
  }

	// bool myGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
	// 			  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
	// 	bool result = global_planner::GlobalPlanner::makePlan(goal, start, plan);

	// 	// Now copy orientation of goal pose to plan
	// 	geometry_msgs::PoseStamped goal_copy = goal;
	// 	goal_copy.header.stamp = ros::Time::now();
	// 	plan[0] = goal_copy;
	// 	std::reverse(plan.begin(), plan.end());

	// 	return result;
	// }
  // void waypointsHandler(const geometry_msgs::PoseStampedConstPtr& msg)
  // {

  // }

	bool myGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
				  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
		bool result;
    if(firstFlag_)
    {
      std::ofstream myfile("/home/xavier/cartographer_ws/paths/mypath.txt",std::ios::app);

      myfile.setf(std::ios::fixed, std::ios::floatfield);
      myfile.precision(5);
      myfile << goal.header.stamp << " "
              << goal.pose.position.x << " "
              << goal.pose.position.y << " "
              << goal.pose.position.z << " "
              << goal.pose.orientation.x << " "
              << goal.pose.orientation.y << " "
              << goal.pose.orientation.z << " "
              << goal.pose.orientation.w << std::endl;
      myfile.close();
      result = global_planner::GlobalPlanner::makePlan(start, goal, plan);
      if(result==false)
      {
        ROS_INFO("Can not arrive this goal position!!!");

        return false;
      }


      lastGoal = goal;
      tempMarker.pose = goal.pose;
      tempMarker.id = waypointID;
      waypointID++;
      mymarkers.markers.push_back(tempMarker);
      waypoints_pub.publish(mymarkers);      
      tempPlan = plan;
      mypath.poses = tempPlan;
      mypath.header.stamp = ros::Time::now();
      tempPath_pub.publish(mypath);
      finalPlan.insert(finalPlan.end(),tempPlan.begin(),tempPlan.end());
      geometry_msgs::PoseStamped lastGoal = goal;
      firstFlag_ = false;
      ROS_INFO("First Planning!!!!");
    }
    else{
      ROS_INFO("Received New Goal");
      std::ofstream myfile("/home/xavier/cartographer_ws/paths/mypath.txt",std::ios::app);

      myfile.setf(std::ios::fixed, std::ios::floatfield);
      myfile.precision(5);
      myfile << goal.header.stamp << " "
              << goal.pose.position.x << " "
              << goal.pose.position.y << " "
              << goal.pose.position.z << " "
              << goal.pose.orientation.x << " "
              << goal.pose.orientation.y << " "
              << goal.pose.orientation.z << " "
              << goal.pose.orientation.w << std::endl;
      myfile.close();
      result = global_planner::GlobalPlanner::makePlan(lastGoal, goal, plan);
      if(result==false)
      {
        ROS_INFO("Can not arrive this goal position!!!");
        // result = true;
      }
      else
      {
        lastGoal = goal;
        tempMarker.pose = goal.pose;
        tempMarker.id = waypointID;
        waypointID++;
        mymarkers.markers.push_back(tempMarker);
        waypoints_pub.publish(mymarkers);
        tempPlan = plan;
        finalPlan.insert(finalPlan.end(),tempPlan.begin(),tempPlan.end());
        // Now copy orientation of goal pose to plan
        geometry_msgs::PoseStamped lastGoal = goal;
        mypath.poses = finalPlan;
        mypath.header.stamp = ros::Time::now();
        tempPath_pub.publish(mypath);
      }


    }
    plan = finalPlan;


		return result;
	}
}

