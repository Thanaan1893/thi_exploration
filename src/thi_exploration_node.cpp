// Including the input and output stream
#include <iostream>
#include <vector>
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


//Klasse Aufbau frontier 
//Gruppierung der Frontier Zellen

class Frontiers {

  private:
    std:: vector< int> pixel_length;

  public:

    Frontiers(){
      cout << "object created" <<endl;

    }
    
    void set_pixel_length( int pos){
      pixel_length.push_back(pos);

    }

    void print_pixel_length(){
      for (int i = 0; i < pixel_length.size(); i++)
      {
        cout<< "Pixel in the Frontiers are: \n"<< pixel_length[i] <<endl;
      }
      
    }


};

// global variables
//!< global variable to publish map
ros::Publisher map_pub;
int free_pixel = 0;
int occupied_pixel = 100;
int unexplored_pixel = -1;
std::vector <Frontiers> every_frontier;

//Function for the received map data
void mapCallback(const nav_msgs:: OccupancyGrid::ConstPtr& msg);


//Function for the Frontier pixels in the right-line
//Parameter std::vector with the mapdata, nav msgs:: mapinfo and the Position of the Pixel
void RightFrontiers( std::vector<signed char> rightdata, nav_msgs::MapMetaData rightinfo , int rightcounter)
{ 
    int right = rightcounter;
    Frontiers rightfront();
    //every_frontier.push_back(rightfront);
    // while (( right % rightinfo.width != rightinfo.width -1 ) && (rightdata[right]!=occupied_pixel)){
    //rightfront.set_pixel_length(right);
    //rightfront.print_pixel_length();
    //right++;
    //} 
    //cout << "End of Frontiers object: \n" << right << endl;
        
        
}

//Function for the Frontier pixels in the left-line
//Parameter std::vector with the mapdata, nav msgs:: mapinfo and the Position of the Pixel
void LeftFrontiers( std::vector<signed char> leftdata, nav_msgs::MapMetaData leftinfo , int leftcounter)
{ 
    int left = leftcounter;
    Frontiers leftfront();
    //every_frontier.push_back(leftfront);
    //while (( left % leftinfo.width != 0 ) && (leftdata[left]!=occupied_pixel)){
    //leftfront.set_pixel_length(left);
    //leftfront.print_pixel_length();
    //left--;
    //} 
    //cout << "End of Frontiers object: \n" << left << endl;
        
        
}

//Function for the Frontier pixels in the up-column
//Parameter std::vector with the mapdata, nav msgs:: mapinfo and the Position of the Pixel
void UpFrontiers( std::vector<signed char> updata, nav_msgs::MapMetaData upinfo , int upcounter)
{ 
    int up = upcounter;
    Frontiers upfront();
    //every_frontier.push_back(upfront);
    //while (( up <= upinfo.width -1 ) && (updata[up]!=occupied_pixel)){
    //upfront.set_pixel_length(up);
    //upfront.print_pixel_length();
    //up = up - upinfo.width;
    //} 
    //cout << "End of Frontiers object: \n" << up << endl;
        
        
}

//Function for the Frontier pixels in the down-column
//Parameter std::vector with the mapdata, nav msgs:: mapinfo and the Position of the Pixel
void DownFrontiers( std::vector<signed char> downdata, nav_msgs::MapMetaData downinfo , int downcounter)
{ 
    int down = downcounter;
    Frontiers downfront();
    //every_frontier.push_back(downfront);
    //while (( down >= downinfo.width * downinfo.height-1 ) && (downdata[down]!=occupied_pixel)){
    //downfront.set_pixel_length(down);
    //downfront.print_pixel_length();
    //down = down + downinfo.width;
    //} 
    //cout << "End of Frontiers object: \n" << down << endl;
        
        
}

//Function for the Frontier pixels in the left-up-diagonal-line
//Parameter std::vector with the mapdata, nav msgs:: mapinfo and the Position of the Pixel
void LeftUpFrontiers( std::vector<signed char> leftupdata, nav_msgs::MapMetaData leftupinfo , int leftupcounter)
{ 
    int leftup = leftupcounter;
    Frontiers leftupfront();
    //every_frontier.push_back(leftupfront);
    //while (( leftup <= leftupinfo.width -1 ) && (leftupdata[leftup]!=occupied_pixel)){
    //leftupfront.set_pixel_length(leftup);
    //leftfront.print_pixel_length();
    //leftup = leftup - leftupinfo.width-1;
    //} 
    //cout << "End of Frontiers object: \n" << leftup << endl;
        
        
}

//Function for the Frontier pixels in the right-up-diagonal-line
//Parameter std::vector with the mapdata, nav msgs:: mapinfo and the Position of the Pixel
void RightUpFrontiers( std::vector<signed char> rightupdata, nav_msgs::MapMetaData rightupinfo , int rightupcounter)
{ 
    int rightup = rightupcounter;
    Frontiers rightupfront();
    //every_frontier.push_back(rightupfront);
    //while (( rightup <= rightupinfo.width -1 ) && (rightupdata[rightup]!=occupied_pixel)){
    //rightupfront.set_pixel_length(rightup);
    //rightfront.print_pixel_length();
    //rightup = rightup - rightupinfo.width+1;
    //} 
    //cout << "End of Frontiers object: \n" << rightup << endl;
        
        
}

//Function for the Frontier pixels in the left-down-diagonal-line
//Parameter std::vector with the mapdata, nav msgs:: mapinfo and the Position of the Pixel
void LeftDownFrontiers( std::vector<signed char> leftdowndata, nav_msgs::MapMetaData leftdowninfo , int leftdowncounter)
{ 
    int leftdown = leftdowncounter;
    Frontiers leftdownfront();
    //every_frontier.push_back(leftdownfront);
    //while (( leftdown >= leftdowninfo.width * leftdowninfo.height-1 ) && leftdowndata[leftdown]!=occupied_pixel)){
    //leftdownfront.set_pixel_length(leftdown);
    //leftdownfront.print_pixel_length();
    //leftdown = leftdown + leftdowninfo.width;
    //} 
    //cout << "End of Frontiers object: \n" << leftdown << endl;
        
        
}

//Function for the Frontier pixels in the Right-up-diagonal-line
//Parameter std::vector with the mapdata, nav msgs:: mapinfo and the Position of the Pixel
void RightDownFrontiers( std::vector<signed char> rightdowndata, nav_msgs::MapMetaData rightdowninfo , int rightdowncounter)
{ 
    int rightdown = rightdowncounter;
    Frontiers rightdownfront();
    //every_frontier.push_back(rightdownfront);
    //while (( rightdown >= rightdowninfo.width * rightdowninfo.height-1 ) && rightdowndata[rightdown]!=occupied_pixel)){
    //rightdownfront.set_pixel_length(rightdown);
    //rightdownfront.print_pixel_length();
    //rightdown = rightdown + rightdowninfo.width;
    //} 
    //cout << "End of Frontiers object: \n" << rightdown << endl;
        
        
}

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
      //ROS_INFO("The neighbours of the up-left corner are  %d, %d \n", i+1 , i+info_old.width);
      if( (msg->data[i] ==free_pixel) && ((msg->data[i+1] ==unexplored_pixel) || (msg->data[i+info_old.width] ==unexplored_pixel) )){
        //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
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
      //ROS_INFO("The neighbours of the up-right corner are  %d, %d \n", i-1,  i+info_old.width);
      if( (msg->data[i] ==free_pixel) && ((msg->data[i-1] ==unexplored_pixel) || (msg->data[i+info_old.width] ==unexplored_pixel))){
        //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
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
        //ROS_INFO("The neighbours in the first line (without up left/right corner) are  %d, %d, %d \n", i-1, i+1 , i+info_old.width);
        if( (msg->data[i] ==free_pixel) && ((msg->data[i-1] ==unexplored_pixel) || (msg->data[i+1] ==unexplored_pixel) || (msg->data[i+info_old.width] ==unexplored_pixel))){
          //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
          NewMap.data[i] =100;
        }
      continue;
      }
    }
    // Pixels in the first column --> not included up-left / down left corner
    if(i % info_old.width == 0){
      if (i!=0) {
        if (i != info_old.width * (info_old.height-1)){
          //ROS_INFO("The neighbours  in the first column (wihtout up/down left corner) are  %d, %d, %d \n", i-info_old.width, i+1, i+info_old.width  );
          if( (msg->data[i] ==free_pixel) &&( (msg->data[i-info_old.width] ==unexplored_pixel) || (msg->data[i+1] ==unexplored_pixel) || (msg->data[i+info_old.width] ==unexplored_pixel))){
            //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
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
      //ROS_INFO("The neighbours of the down-left corner are  %d, %d \n", i-info_old.width, i+1  );

      if( (msg->data[i] ==free_pixel) && ((msg->data[i-info_old.width] ==unexplored_pixel) || (msg->data[i+1] ==unexplored_pixel)) ){
        //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
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
        //ROS_INFO("The neighbours in the last line (without down-left/right corner) are  %d, %d, %d \n", i-1,  i-info_old.width, i+1 );
        
        if( (msg->data[i] ==free_pixel) && ((msg->data[i-1] ==unexplored_pixel) || (msg->data[i-info_old.width] ==unexplored_pixel) || (msg->data[i+1] ==unexplored_pixel) ) ){
            //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
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
        //ROS_INFO("The neighbours in the last column (without up/down right corner) are  %d, %d, %d \n", i-info_old.width, i-1, i+info_old.width  );
        
        if( (msg->data[i] ==free_pixel) && ((msg->data[i-info_old.width] ==unexplored_pixel) || (msg->data[i-1] ==unexplored_pixel) ||  (msg->data[i+1] ==unexplored_pixel) )){
          //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
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
      //ROS_INFO_STREAM("The neighbours are of the down-right corner " << i-1 << " " << i-info_old.width << "\n");
      
      if( (msg->data[i] ==free_pixel) && ((msg->data[i-1] ==unexplored_pixel) || (msg->data[i-info_old.width] ==unexplored_pixel) )){
        //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
        NewMap.data[i] =100;
        
        
      }

      else
      {
        NewMap.data[i] =0;
      }
      
      continue;
    }

    //Other Pixels
   // ROS_INFO_STREAM("The neighbours are "<< i-1 <<" " << i-info_old.width <<" "<< i+1 <<" "<< i+info_old.width<<" "<<  "\n"  );
    if( (msg->data[i] ==free_pixel) &&( (msg->data[i-1] == unexplored_pixel) || (msg->data[i-info_old.width] == unexplored_pixel) ||  (msg->data[i+1] ==unexplored_pixel) || (msg->data[i+info_old.width] ==unexplored_pixel) )){
      //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
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



