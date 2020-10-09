// Including the input and output stream
#include <iostream>

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

using namespace std;

// global variables
//!< global variable to publish map
ros::Publisher map_pub;

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

  // Initialization for the ROS-Subscriber to subscribe the map data
  ros::Subscriber map_sub = nh.subscribe("gmapping/map", 1, mapCallback);

  // Initialization for the ROS-Publisher to publish the new map data
  map_pub = nh.advertise<nav_msgs::OccupancyGrid>("frontier_exploration",1);


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
  
  ros::spin(); 
  
  return 0;
  
}

//Function for the received map data with the parameter nav_msgs --> OccupancyGrid
void mapCallback(const nav_msgs:: OccupancyGrid::ConstPtr& msg)
{ 
  //Saving the parts of the OccupancyGrid in variables --> see documentation: doc.ros.org
  std_msgs::Header header_old = msg->header;
  nav_msgs::MapMetaData info_old = msg->info;
  std::vector<signed char> data = msg->data;

  nav_msgs:: OccupancyGrid NewMap;
  NewMap.header = header_old;  
  NewMap.info = info_old;
  NewMap.data = msg->data;

  //Show the height, width and size of the map
  ROS_INFO("Got map %d, %d", info_old.width, info_old.height);
  ROS_INFO_STREAM("size of map " << data.size());

  // 1. routine für abfrage der vier nachbarn schreiben
  // 2. schleife über alle zellen
  // 3. publisher für neue karte
  // 4. überlegen wie grenzen gefunden werden können (zellebene)
  for (int i =0; i < info_old.width *info_old.height; i++) {

    //Pixel at the up-left corner
    if(i==0){
      ROS_INFO("The neighbours of the up-left corner are  %d, %d \n", i+1 , i+info_old.width);
      if( (msg->data[i] ==100) || (msg->data[i+1] ==100) || (msg->data[i+info_old.width] ==100) ){
        ROS_INFO_STREAM("pixel" << i <<" is a boundary");
        NewMap.data[i] =100;
      }

      else
      {
        NewMap.data[i] =0;
      }
      
      continue;
    }

    //Pixel at the up-right corner
    if(i==info_old.width-1){
      ROS_INFO("The neighbours of the up-right corner are  %d, %d \n", i-1,  i+info_old.width);
      if( (msg->data[i] ==100) || (msg->data[i-1] ==100) || (msg->data[i+info_old.width] ==100)){
        ROS_INFO_STREAM("pixel" << i <<" is a boundary");
        NewMap.data[i] ==100;
      }
      else
      {
        NewMap.data[i] ==0;
      }
      
      continue;
    }
       
    //Pixels in the first line--> not included the up left/ up-right corner
    if(i<info_old.width-1){
      if(i!=0){
        ROS_INFO("The neighbours in the first line (without up left/right corner) are  %d, %d, %d \n", i-1, i+1 , i+info_old.width);
        if( (msg->data[i] ==100) || (msg->data[i-1] ==100) || (msg->data[i+1] ==100) || (msg->data[i+info_old.width] ==100)){
          ROS_INFO_STREAM("pixel" << i <<" is a boundary");
          NewMap.data[i] =100;
        }
      continue;
      }
    }
    // Pixels in the first column --> not included up-left / down left corner
    if(i % info_old.width == 0){
      if (i!=0) {
        if (i != info_old.width * (info_old.height-1)){
          ROS_INFO("The neighbours  in the first column (wihtout up/down left corner) are  %d, %d, %d \n", i-info_old.width, i+1, i+info_old.width  );
          if( (msg->data[i] ==100) || (msg->data[i-info_old.width] ==100) || (msg->data[i+1] ==100) || (msg->data[i+info_old.width] ==100)){
            ROS_INFO_STREAM("pixel" << i <<" is a boundary");
            NewMap.data[i] =100;
          }

          else
          {
            NewMap.data[i] =0;
          }
          
          continue;
        }
      }
    }
    
  
    // Pixel at the down-left corner
     if(i == info_old.width * (info_old.height-1)){
      ROS_INFO("The neighbours of the down-left corner are  %d, %d \n", i-info_old.width, i+1  );

      if( (msg->data[i] ==100) || (msg->data[i-info_old.width] ==100) || (msg->data[i+1] ==100) ){
        ROS_INFO_STREAM("pixel" << i <<" is a boundary");
         NewMap.data[i] =100;
      }

      else
      {
        NewMap.data[i] =0;
      }
      
      continue;
      }

    // Pixels in the last line--> not included the down-left / down right corner
    if(i>info_old.width *(info_old.height-1)) {
      if (i< info_old.width *info_old.height-1){
        ROS_INFO("The neighbours in the last line (without down-left/right corner) are  %d, %d, %d \n", i-1,  i-info_old.width, i+1 );
        
        if( (msg->data[i] ==100) || (msg->data[i-1] ==100) || (msg->data[i-info_old.width] ==100) || (msg->data[i+1] ==100) ){
            ROS_INFO_STREAM("pixel" << i <<" is a boundary");
            NewMap.data[i] =100;
        }

        else
        {
          NewMap.data[i] =0;
        }
        
        continue;
      }
    }

    // Pixels in the last column --> not included the up-right / down right corner
    if (i % info_old.width == info_old.width -1) {
      if (i!=info_old.width-1) {
        if (i != info_old.width * info_old.height -1){
        ROS_INFO("The neighbours in the last column (without up/down right corner) are  %d, %d, %d \n", i-info_old.width, i-1, i+info_old.width  );
        
        if( (msg->data[i] ==100) || (msg->data[i-info_old.width] ==100) || (msg->data[i-1] ==100) ||  (msg->data[i+1] ==100) ){
          ROS_INFO_STREAM("pixel" << i <<" is a boundary");
          NewMap.data[i] =100;
        }

        else
        {
          NewMap.data[i] =0;
        }
        
        continue;
        }
      }
    }

    //Pixel at the down-right corner
    if(i == info_old.width * info_old.height -1){
      ROS_INFO_STREAM("The neighbours are of the down-right corner " << i-1 << " " << i-info_old.width << "\n");
      
      if( (msg->data[i] ==100) || (msg->data[i-1] ==100) || (msg->data[i-info_old.width] ==100) ){
        ROS_INFO_STREAM("pixel" << i <<" is a boundary");
        NewMap.data[i] =100;
      }

      else
      {
        NewMap.data[i] =0;
      }
      
      continue;
    }

    //Other Pixels
    ROS_INFO_STREAM("The neighbours are "<< i-1 <<" " << i-info_old.width <<" "<< i+1 <<" "<< i+info_old.width<<" "<<  "\n"  );
    if( (msg->data[i] ==0) &&( (msg->data[i-1] ==-1) || (msg->data[i-info_old.width] ==-1) ||  (msg->data[i+1] ==-1) || (msg->data[i+info_old.width] ==-1) )){
      ROS_INFO_STREAM("pixel" << i <<" is a boundary");
      NewMap.data[i] =100;
    }

    else
    {
      NewMap.data[i] =0;
    }
    
    
  }  
  // Initialization for the ROS-Publisher to publish the new map data
  ROS_INFO_STREAM("Publishing the NewMap.\n"  );

  map_pub.publish(NewMap);
}



//Klasse Aufbau frontier 
//Gruppierung der Frontier Zellen
