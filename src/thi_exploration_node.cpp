// Including the ROS libary
#include <ros/ros.h>

//Including Move-Base and action libary-->Navigtion Stack  
#include <move_base/move_base.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>

//Including Standard- and Navigation Massages, because OccupancyGrid is using them  
#include <std_msgs/Header.h>
#include <nav_msgs/MapMetaData.h>

//Including Navigation Massages --> OccupancyGrid
#include <nav_msgs/OccupancyGrid.h>

// Creating a convenience typedef for a SimpleActionClient; For communication with the MoveBaseAction action interface.
// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//Function for the received map data
void mapCallback(const nav_msgs:: OccupancyGrid::ConstPtr& msg);

/**
 * Main function of ros node
 * @param argc
 * @param argv
 * @return
 */

int main( int argc, char ** argv)
{ 
  // Initialization of ros node
  ros::init(argc, argv , "thi_exploration");
   // Initialization of node handle for communication with ROSCORE
  ros::NodeHandle nh;

 // Initialization for the ROS-Publisher to publish the map data
  ros::Subscriber map_sub = nh.subscribe("map", 2000, mapCallback);



  //MoveBaseClient constructor(communication with MoveBaseAction action interface)
  //Telling the action client to start a thread to call ros::spin()
  //ROS callbacks will be processed by passing "true" as the second argument of the MoveBaseClient constructor.
  // MoveBaseClient ac("move_base",true);

  //wait for the action server to come up
  // while (!ac.waitForServer(ros::Duration(5.0)))
  // {
  //   ROS_INFO("Wait for the move_base action server");
  // }
  
  // move_base_msgs::MoveBaseGoal goal;

  // //Sending a goal to the robot
  // goal.target_pose.header.frame_id = "map";
  // goal.target_pose.header.stamp = ros::Time::now();

  // goal.target_pose.pose.position.x = 4.0;
  // goal.target_pose.pose.position.y = 13.0;
  // goal.target_pose.pose.orientation.w = 1.0;
  
  // ROS_INFO("Sending goal");
  // ac.sendGoal(goal);
  
  // ac.waitForResult();

  // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  //   ROS_INFO("Moving was correct");
  // else
  //   ROS_INFO("Moving failed");
  
  
  return 0;
  
}

//Function for the received map data with the parameter nav_msgs --> OccupancyGrid
void mapCallback(const nav_msgs:: OccupancyGrid::ConstPtr& msg)
{ 
  //Saving the parts of the OccupancyGrid in variables --> see documentation: doc.ros.org
  std_msgs:: Header header = msg->header;
  nav_msgs:: MapMetaData info = msg->info;
  int data []  = msg->data;
 
  //Show the height and width of the map
  ROS_INFO("Got map %d, %d", info.width, info.height);

  //Checking of the first pixel is unexplored
  if(msg->data[0] ==-1){
    ROS_INFO("pixel is unexplored");
    }

}