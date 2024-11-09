/*
 * navfn_ros_ext.h
 *
 *  Created on: Nov 28, 2016
 *      Author: thobotics
 */

#ifndef MY_GLOBAL_PLANNER
#define MY_GLOBAL_PLANNER
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <global_planner/planner_core.h>
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
namespace navfn {
/**
   * @class NavfnROS
   * @brief Provides a ROS wrapper for the navfn planner which runs a fast, interpolated navigation function on a costmap.
   */
	class myGlobalPlanner : public global_planner::GlobalPlanner {


	public:
		myGlobalPlanner();
		myGlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap_ros, std::string frame_id);
		/**
		 * Note: it is overload function
		 */
		void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

		/**
		 * This is important !!!. Move_base call this, need to force this
		 * call swapped makePlan
		 */
		bool makePlan(const geometry_msgs::PoseStamped& start,
				const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

		void waypointsHandler(const geometry_msgs::PoseStampedConstPtr& msg);
	private:
		ros::NodeHandle nh;
		float markerScale;
		ros::Subscriber waypoint_sub;
		ros::Publisher tempPath_pub;
		ros::Publisher waypoints_pub;
		visualization_msgs::Marker tempMarker;
		visualization_msgs::MarkerArray mymarkers;
		int waypointID;
		geometry_msgs::PoseStamped lastGoal;
		nav_msgs::Path mypath;
		bool firstFlag_;
		bool initialized_;
		std::string global_frame_id;
		std::vector<geometry_msgs::PoseStamped> tempPlan;
		std::vector<geometry_msgs::PoseStamped> finalPlan;


	};
};



#endif /* TURTLEBOT_MPEPC_INCLUDE_TURTLEBOT_MPEPC_NAVFN_ROS_EXT_H_ */
