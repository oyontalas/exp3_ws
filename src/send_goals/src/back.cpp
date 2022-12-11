#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include<iostream>
using namespace std;
// 备份文件
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char** argv){
    ros::init(argc, argv, "send_goals_node");
    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer( ros::Duration( 5.0 ) )){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;
    ROS_INFO(" Init success!!! ");
    // 第一个待发送的 目标点 在 map 坐标系下的坐标位置
    goal.target_pose.pose.position.x = 2.77;
    goal.target_pose.pose.position.y =  -2.88;
    goal.target_pose.pose.orientation.z =  -0;  
    goal.target_pose.pose.orientation.w = 0.951839761956;  

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal);
    ROS_INFO("send goal");
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("The Goal achieved success !!!" );
    }
    else{
        ROS_WARN("The Goal Planning Failed for some reason"); 
    }
  return 0;
}