#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//使用Action通信所需要的头文件
#include <iomanip>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; 

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "MoveBaseAction");
    ros::NodeHandle n;
    

    std::ifstream pathFile("/home/xavier/cartographer_ws/paths/mypath.txt");
    if(!pathFile)
    {
        ROS_INFO("Can not find the path file!!!");
        return 0;
    }
    std::string line;
    geometry_msgs::PoseStamped tempWaypoint;
    std::string tempNum;
    float lastTime;
    std::vector<geometry_msgs::PoseStamped> waypoints;
    while(getline(pathFile,line))
    {
        istringstream readstr(line);
        getline(readstr,tempNum,' ');
 
        getline(readstr,tempNum,' ');
        std::cout<<tempNum<<" "; 
        tempWaypoint.pose.position.x = atof(tempNum.c_str());
        getline(readstr,tempNum,' ');
        std::cout<<tempNum<<" ";   

        tempWaypoint.pose.position.y = atof(tempNum.c_str());
        getline(readstr,tempNum,' ');
        std::cout<<tempNum<<" ";   

        tempWaypoint.pose.position.z = atof(tempNum.c_str());
        getline(readstr,tempNum,' ');
        std::cout<<tempNum<<" ";   

        tempWaypoint.pose.orientation.x = atof(tempNum.c_str());
        getline(readstr,tempNum,' ');
        std::cout<<tempNum<<" ";   

        tempWaypoint.pose.orientation.y = atof(tempNum.c_str());
        getline(readstr,tempNum,' ');
        std::cout<<tempNum<<" ";   

        tempWaypoint.pose.orientation.z = atof(tempNum.c_str());
        getline(readstr,tempNum,' ');
        std::cout<<tempNum<<" "<<std::endl;   

        tempWaypoint.pose.orientation.w = atof(tempNum.c_str());
        tempWaypoint.header.frame_id = "map";
        waypoints.push_back(tempWaypoint);

    }
    pathFile.close();



    //tell the action client that we want to spin a thread by default
    //don't need ros::spin() 创建action客户端，参数1：action名，参数2：true，不需要手动调用ros::spin()，会在它的线程中自动调用。
	MoveBaseClient ac("move_base", true);
	 
	//wait for the action server to come up1000
	while(!ac.waitForServer(ros::Duration(5.0))){
	ROS_INFO("Waiting for the move_base action server to come up");
	}
	
	// 自己初始化一个坐标吧
	geometry_msgs::PoseStamped send_Pose;
    move_base_msgs::MoveBaseGoal goal;
    for(auto waypoint:waypoints)
    {
        goal.target_pose = waypoint;//此处记得要定义MoveBaseGoal类的goal
        goal.target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(goal);
        ac.waitForResult();
        // ac.waitForResult(ros::Duration(30.0));
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("already reached goal!");
            //TODO: 成功到达目的地，此处发挥想象做点啥
            sleep(5);
        }
        else{
            ROS_INFO("Failed to reach the goal!");  
        }  
    }
 
	
    /*ros::Rate loop_rate(10);
    ros::spin();*/

    return 0;
}
