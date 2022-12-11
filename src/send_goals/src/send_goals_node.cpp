#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include<iostream>
#include<geometry_msgs/PoseStamped.h>
#include<string.h>
using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient ac("move_base", true);
move_base_msgs::MoveBaseGoal goal;

void get_goal(const geometry_msgs::PoseStamped::ConstPtr pose_p){
    goal.target_pose.pose.position.x = pose_p->pose.position.x;
    goal.target_pose.pose.position.y = pose_p->pose.position.y;
    goal.target_pose.pose.orientation.z =  0;  
    goal.target_pose.pose.orientation.w = 0.951839761956;  
    goal.target_pose.header.frame_id = "map";
    std::cout<<goal<<std::endl;

    }

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char** argv){
    ros::init(argc, argv, "send_goals_node");
    MoveBaseClient ac("move_base", true);
    ros::NodeHandle n; 
    bool got = false;
    bool change = false;



ros::Subscriber sub_ = n.subscribe("/py2cpp_goal", 10, get_goal);
    ROS_INFO(" Init success!!! ");
    while(ros::ok){
        while(!ac.waitForServer( ros::Duration( 5.0 ) )){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        std::cout<<got<<std::endl;
        if(!got) continue;
        if(!change) continue;
        change = false;
        goal.target_pose.header.stamp = ros::Time::now();
        // ac.sendGoal(get_goal.goal);
        // ROS_INFO(string(get_goal.goal));
        std::cout<<goal<<std::endl;
        break;
        ROS_INFO("send goal");
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("The Goal achieved success !!!" );
        }
        else{
            ROS_WARN("The Goal Planning Failed for some reason"); 
        }
    }

  return 0;
}