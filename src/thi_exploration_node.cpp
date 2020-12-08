// Including the input and output stream
#include <iostream>

// Including standard vector
#include <vector>

// Including the ROS libary
#include <ros/ros.h>

// Including the tf libary, transforamtiontree, 3-D-Point-Vector
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>

// Including the visualization-tools in RVIZ
#include <visualization_msgs/Marker.h>

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
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


using namespace std;


//Klasse Aufbau frontier 
//Gruppierung der Frontier Zellen

class Frontiers {

  private:
    std:: vector <int> pixels;
    std:: vector <float> xpos;
    std:: vector <float> ypos;
    std:: vector <tf::Vector3> orientation_pixels;
    int number_of_pixel;
    float gravity_of_center_x;
    float gravity_of_center_y;
    float distance;
    float orientation_frontier_x;
    float orientation_frontier_y;

  public:

    Frontiers(){
      number_of_pixel = 0;
      gravity_of_center_x = 0;
      gravity_of_center_y = 0;
      distance = 0;
      orientation_frontier_x = 0;
      orientation_frontier_y = 0;

    }

    void delete_frontier(){
      pixels.clear();
      xpos.clear();
      ypos.clear();
      orientation_pixels.clear();
      number_of_pixel = 0;
      gravity_of_center_x = 0;
      gravity_of_center_y = 0;
      distance = 0;
      orientation_frontier_x = 0;
      orientation_frontier_y = 0;
    }
    
    void set_pixels( int pos){
      pixels.push_back(pos);

    }

    void print_frontier(){
      for (int i = 0; i < pixels.size(); i++)
      {
        cout<< "Pixel in the Frontiers are: "<< pixels[i] <<endl;
        cout<< "Pixel in x coordinate: "<< xpos[i] <<endl;
        cout<< "Pixel in y_coordinate: "<< ypos[i] <<endl;
      }
        cout<< "gravity_of_center x coordinate: "<< gravity_of_center_x <<endl;
        cout<< "gravity_of_center y_coordinate: "<< gravity_of_center_y <<endl;
    }
    
    bool pixel_item_exists(int number){
      for (int j = 0; j < pixels.size(); j++)
      { 
        if(pixels[j]==number){
          
          return true;
        }
      }
      
      return false;
    }

    int get_pixel_size(){
      number_of_pixel = pixels.size();
      return number_of_pixel;
    }
  
    int get_pixel(int index)
  {
    return pixels[index];
  }

   float get_gravity_of_center_x()
  {
    return gravity_of_center_x;
  }

   float get_gravity_of_center_y()
  {
    return gravity_of_center_y;
  }

  float get_distance()
  {
    return distance;
  }

  float get_orientation_frontier_x()
  {
    return orientation_frontier_x;
  }
  float get_orientation_frontier_y()
  {
    return orientation_frontier_y;
  }

  void pixels_x_y_transformation (nav_msgs::MapMetaData info_x){
    float x = 0;
    float y = 0;
    for (int j = 0; j < pixels.size(); j++){
      x = pixels[j] % info_x.width * info_x.resolution + info_x.origin.position.x;
      y =  pixels[j]  / info_x.width * info_x.resolution + info_x.origin.position.y;
      xpos.push_back(x);
      ypos.push_back(y);

    }
  }

   void calc_gravity_of_center (){
    float x = 0;
    float y = 0;
    for (int j = 0; j < xpos.size(); j++){
      x = x + xpos[j];
      y = y + ypos[j];
    }
    gravity_of_center_x = x/xpos.size();
    gravity_of_center_y = y/ypos.size();
  }

  void euclidean_distance (float current_x, float current_y, float future_x , float future_y){
    distance = sqrt( (current_x-future_x) * (current_x-future_x) + (current_y - future_y) * (current_y - future_y) );

  } 

  void calc_orientation_pixel(int latest, std::vector<signed char> data_orientation, nav_msgs::MapMetaData info_orientation )
  {
    tf::Vector3 latest_orientation (0,0,0);
    int unexplored_pixel = -1;
    
    //Right
    if( (latest % info_orientation.width != info_orientation.width -1) && (data_orientation[latest+1]== unexplored_pixel) ){
      latest_orientation = latest_orientation + tf::Vector3 (1,0,0);
    }

    //Left
    if( (latest % info_orientation.width != 0) && (data_orientation[latest-1]== unexplored_pixel) ){
      latest_orientation = latest_orientation + tf::Vector3 (-1,0,0);
    }

    //Up   
    if( (latest>= info_orientation.width -1) && (data_orientation[latest-info_orientation.width]== unexplored_pixel) ){
      latest_orientation = latest_orientation + tf::Vector3 (0,-1,0);
    }

    //Down
    if( (latest<= info_orientation.width * info_orientation.height-1) && (data_orientation[latest+info_orientation.width]== unexplored_pixel) ){
      latest_orientation = latest_orientation + tf::Vector3 (0,1,0);
    }

    //LeftUp
    if( (latest >= info_orientation.width -1) && (latest % info_orientation.width != 0) && (data_orientation[latest-info_orientation.width-1]== unexplored_pixel) ){
      latest_orientation = latest_orientation + tf::Vector3 (-1,-1,0);
    }

    //RightUp
    if( (latest >= info_orientation.width -1) && (latest % info_orientation.width != info_orientation.width -1) && (data_orientation[latest-info_orientation.width+1]== unexplored_pixel) ){
      latest_orientation = latest_orientation + tf::Vector3 (1,-1,0);
    }

    //LeftDown
    if( (latest <= info_orientation.width * info_orientation.height-1) && (latest % info_orientation.width != 0) && (data_orientation[latest+info_orientation.width-1]== unexplored_pixel) ){
      latest_orientation = latest_orientation + tf::Vector3 (-1,1,0);
    }

    //RightDown
    if( (latest <= info_orientation.width * info_orientation.height-1) && (latest % info_orientation.width != info_orientation.width -1)&& (data_orientation[latest+info_orientation.width+1]== unexplored_pixel) ){
      latest_orientation = latest_orientation + tf::Vector3 (1,1,0);
    }
    
    //cout << "x: " << latest_orientation[0] << " y: " << latest_orientation[1] <<" z: " << latest_orientation[2] << endl;
    orientation_pixels.push_back(latest_orientation);
  }

  void calc_orientation_frontier(){
    float sum_x = 0;
    float sum_y = 0;
    for (int j = 0; j < orientation_pixels.size(); j++){
      sum_x = sum_x + orientation_pixels[j][0];
      sum_y = sum_y + orientation_pixels[j][1];
    }
    sum_x = sum_x/ orientation_pixels.size();
    sum_y = sum_y/ orientation_pixels.size();

    orientation_frontier_x = sum_x;
    orientation_frontier_y = sum_y;
  }

};

class Pixel {
   
  private:

  int identifier;

  public:
    Pixel(){
      identifier = 0;
    }

    void check_pixeltype(int currentpos, std::vector<signed char> data_current, nav_msgs::MapMetaData info_current){
      int free_pixel = 0;
      int unexplored_pixel = -1;

      // Up - left Corner
      if(currentpos == 0){

        if( (data_current[currentpos] ==free_pixel) && ( (data_current[currentpos+1] == unexplored_pixel) || (data_current[currentpos+info_current.width] == unexplored_pixel) ) ){

          identifier = 1;
        }
      }

      //First line
      else if( (currentpos<info_current.width-1)&&(currentpos!=0) ){

        if( (data_current[currentpos] == free_pixel) && ((data_current[currentpos-1] ==unexplored_pixel) || (data_current[currentpos+1] ==unexplored_pixel) || (data_current[currentpos+info_current.width] ==unexplored_pixel) ) ){

          identifier = 2;
        }
      }

      // Up - right Corner
      else if(currentpos == info_current.width-1){

        if( (data_current[currentpos] == free_pixel) && ((data_current[currentpos-1] == unexplored_pixel) || (data_current[currentpos+info_current.width] ==unexplored_pixel) ) ){

          identifier = 3;
        }
      }
      
      // First Column
      else if( (currentpos % info_current.width == 0) && (currentpos!=0) && (currentpos != info_current.width * (info_current.height-1) ) ){

        if  ( (data_current[currentpos] ==free_pixel) &&( (data_current[currentpos-info_current.width] ==unexplored_pixel) || (data_current[currentpos+1] == unexplored_pixel) || (data_current[currentpos+info_current.width] == unexplored_pixel))){

          identifier = 4;
        }

      }

      // Last Column
      else if( (currentpos % info_current.width == info_current.width -1) && (currentpos!=info_current.width-1) && (currentpos != info_current.width * info_current.height -1)){

        if( (data_current[currentpos] ==free_pixel) && ((data_current[currentpos-info_current.width] ==unexplored_pixel) || (data_current[currentpos-1] ==unexplored_pixel) ||  (data_current[currentpos+1] ==unexplored_pixel) )){

          identifier = 6;
        }
      }

      // Down - left Corner
      else if(currentpos == info_current.width * (info_current.height-1)){
    
        if( (data_current[currentpos] == free_pixel) && ( (data_current[currentpos-info_current.width] == unexplored_pixel) || (data_current[currentpos+1] == unexplored_pixel))){

          identifier = 7;
        }
      }

      // Last Line
      else if( (currentpos >info_current.width *(info_current.height-1)) && (currentpos< info_current.width * info_current.height-1) ){

        if( (data_current[currentpos] ==free_pixel) && ((data_current[currentpos-1] ==unexplored_pixel) || (data_current[currentpos-info_current.width] ==unexplored_pixel) || (data_current[currentpos+1] ==unexplored_pixel) ) ){

          identifier = 8;
        }
      }

      // Down - right Corner
      else if (currentpos == info_current.width * info_current.height-1 ){

        if( (data_current[currentpos] == free_pixel) && ((data_current[currentpos-1] == unexplored_pixel) || (data_current[currentpos-info_current.width] == unexplored_pixel) ) ){

          identifier = 9;
        }
      }
      
       // Center
      else 
      {
        if( (data_current[currentpos]==free_pixel)  && ( (data_current[currentpos-1] == unexplored_pixel) || 
        (data_current[currentpos+1] == unexplored_pixel) || (data_current[currentpos - info_current.width]==unexplored_pixel)|| 
        (data_current[currentpos + info_current.width]==unexplored_pixel)  ) )
        {

          identifier = 5;
        }
      }

    }

    void delete_pixel(){
      identifier = 0;
    }
    
    int get_identifier(){
      return identifier;
    }
};


class MoveBase{

  private:
    move_base_msgs::MoveBaseGoal goal;


  public:
  MoveBase( float position_x, float position_y, float position_z, float orientation_w, float orientation_x, float orientation_y, float orientation_z ){
 

    
    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = position_x; 
    goal.target_pose.pose.position.y = position_y;
    goal.target_pose.pose.position.z = position_z;

    goal.target_pose.pose.orientation.w = orientation_w;
    goal.target_pose.pose.orientation.x = orientation_x;
    goal.target_pose.pose.orientation.y = orientation_y;
    goal.target_pose.pose.orientation.z = orientation_z;

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
   
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved 1 meter forward");
    else
      ROS_INFO("The base failed to move forward 1 meter for some reason");

  
  }
  
  void set_position_x(move_base_msgs::MoveBaseGoal goal, float position_x)
  {
    goal.target_pose.pose.position.x = position_x;
  }

  void set_position_y(move_base_msgs::MoveBaseGoal goal, float position_y)
  {
    goal.target_pose.pose.position.y = position_y;
  }
  void set_position_z(move_base_msgs::MoveBaseGoal goal, float position_z)
  {
    goal.target_pose.pose.position.z = position_z;
  }
  void set_orientation_w(move_base_msgs::MoveBaseGoal goal, float orientation_w)
  {
    goal.target_pose.pose.orientation.w = orientation_w;
  }

  void set_orientation_x(move_base_msgs::MoveBaseGoal goal, float orientation_x)
  {
    goal.target_pose.pose.orientation.x = orientation_x;
  }

  void set_orientation_y(move_base_msgs::MoveBaseGoal goal, float orientation_y)
  {
    goal.target_pose.pose.orientation.y = orientation_y;
  }

  void set_orientation_z(move_base_msgs::MoveBaseGoal goal, float orientation_z)
  {
    goal.target_pose.pose.orientation.z = orientation_z;
  }
  
  void send_coor(move_base_msgs::MoveBaseGoal goal){

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
   
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved 1 meter forward");
    else
      ROS_INFO("The base failed to move forward 1 meter for some reason");

  }
    
};

// global variables
//!< global variable to publish map
ros::Publisher map_pub;
nav_msgs:: OccupancyGrid FrontierMap;
//ros::Publisher vis_pub;

//Function for the received map data with the parameter nav_msgs --> OccupancyGrid
void mapCallback(const nav_msgs:: OccupancyGrid::ConstPtr& msg);
void sendingcoor(int current, std::vector <Frontiers> every_frontier);


bool ExistedFrontier (int position, std::vector <Frontiers> every_frontier){
     
  for (int iter = 0; iter < every_frontier.size(); iter++){
    if( every_frontier[iter].pixel_item_exists(position)== true){
      //cout<< "Pixel is in Every Frontiers " << endl;
      return true;
    } 
     
  }
  //cout<< "Pixel is not in Every Frontiers " << endl;
  return false;
}

int BiggestFrontier (std::vector <Frontiers> every_frontier){

  int size_of_frontier = 0;
  int frontier_pos = 0;
  for (int iter = 0; iter < every_frontier.size(); iter++)
  {
    if( every_frontier[iter].get_pixel_size() > size_of_frontier)
    {
      size_of_frontier=every_frontier[iter].get_pixel_size();
      frontier_pos = iter;
    } 
     
  }
    return frontier_pos;
}

int ShortestDistanceFrontier (std::vector <Frontiers> every_frontier){

  int distance_of_frontier = every_frontier[0].get_distance();
  int pos_frontier = 0;
  for (int iter = 0; iter < every_frontier.size(); iter++)
  {
    if( every_frontier[iter].get_distance() < distance_of_frontier)
    { 
      distance_of_frontier=every_frontier[iter].get_distance();
      pos_frontier = iter;
    } 
     
  }
    return pos_frontier;
}

Frontiers FloodfillFrontiers(int counter, int substitution, std::vector<signed char> data_map, nav_msgs::MapMetaData info_map,  Frontiers transfer ){
  
  int free_pixel = 0;
  int unexplored_pixel = -1;

  int right = counter + 1;
  int left = counter - 1;
  int up = counter - info_map.width;
  int down = counter + info_map.width;
  int leftup = counter - info_map.width-1;
  int rightup = counter  - info_map.width+1;
  int leftdown = counter + info_map.width-1;
  int rightdown = counter  + info_map.width+1;

  Pixel right_cell;
  right_cell.check_pixeltype(right, FrontierMap.data, FrontierMap.info);
  
  Pixel left_cell ;
  left_cell.check_pixeltype(left, FrontierMap.data, FrontierMap.info);

  Pixel up_cell ;
  up_cell.check_pixeltype(up, FrontierMap.data, FrontierMap.info);

  Pixel down_cell ;
  down_cell.check_pixeltype(down, FrontierMap.data, FrontierMap.info);

  Pixel leftup_cell ;
  leftup_cell.check_pixeltype(leftup, FrontierMap.data, FrontierMap.info);

  Pixel rightup_cell ;
  rightup_cell.check_pixeltype(rightup, FrontierMap.data, FrontierMap.info);

  Pixel leftdown_cell ;
  leftdown_cell.check_pixeltype(leftdown, FrontierMap.data, FrontierMap.info);

  Pixel rightdown_cell ;
  rightdown_cell.check_pixeltype(rightdown, FrontierMap.data, FrontierMap.info);
  
  Frontiers collectedfrontiers = transfer;
  collectedfrontiers.set_pixels(counter);

  if (FrontierMap.data[counter] == substitution) {
    //cout << "Wenn die gewünschte Farbe im Pixel schon ersetzt wurde" << endl;
    return collectedfrontiers;
  }

  else if  ( (FrontierMap.data[counter] != free_pixel ) && (FrontierMap.data[counter] != unexplored_pixel ) ){
    //cout << "Wenn gesuchte Farbe nicht im Pixel steht" << endl;
    return collectedfrontiers;
  }
  
  else { 
    //cout << "Ersetze im Pixel die gewünschte Farbe" << endl;
    FrontierMap.data[counter] = substitution;
   }
  
  if  ( (right % info_map.width != info_map.width -1)  ) { 

    if (  (right_cell.get_identifier() == 5) || (right_cell.get_identifier() == 1) || (right_cell.get_identifier() == 2) || 
          (right_cell.get_identifier() == 4) || (right_cell.get_identifier() == 7) || (right_cell.get_identifier() == 8)   ){

      //cout << "rechts" << endl;
      collectedfrontiers.calc_orientation_pixel(right,  FrontierMap.data, FrontierMap.info );
      collectedfrontiers=FloodfillFrontiers(right, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }
  
  
  if  ( left % info_map.width != 0 ) { 

    if (  (left_cell.get_identifier() == 5) || (left_cell.get_identifier() == 2) || (left_cell.get_identifier() == 3) || 
          (left_cell.get_identifier() == 6) || (left_cell.get_identifier() == 8) || (left_cell.get_identifier() == 9)   ){

      //cout << "links" << endl;
      collectedfrontiers.calc_orientation_pixel(left,  FrontierMap.data, FrontierMap.info );
      collectedfrontiers=FloodfillFrontiers(left, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }
  

  if  ( up >= info_map.width -1 )   {
    
    if (  (up_cell.get_identifier() == 5) || (up_cell.get_identifier() == 4) || (up_cell.get_identifier() == 6) || 
          (up_cell.get_identifier() == 7) || (up_cell.get_identifier() == 8) || (up_cell.get_identifier() == 9)   ){

      //cout << "oben" << endl;
      collectedfrontiers.calc_orientation_pixel(up,  FrontierMap.data, FrontierMap.info );
      collectedfrontiers=FloodfillFrontiers(up, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }
  

  if   ( down <= info_map.width * info_map.height-1 )  { 

    if (  (down_cell.get_identifier() == 5) || (down_cell.get_identifier() == 1) || (down_cell.get_identifier() == 2) || 
          (down_cell.get_identifier() == 3) || (down_cell.get_identifier() == 4) || (down_cell.get_identifier() == 6)   ){

      //cout << "unten" << endl;
      collectedfrontiers.calc_orientation_pixel(down,  FrontierMap.data, FrontierMap.info );
      collectedfrontiers=FloodfillFrontiers(down, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }
 

  if  ( leftup >= info_map.width -1 )  { 

    if (  (leftup_cell.get_identifier() == 5) || (leftup_cell.get_identifier() == 6) ||  
          (leftup_cell.get_identifier() == 8) || (leftup_cell.get_identifier() == 9)   ){

      //cout << "linksoben" << endl;
      collectedfrontiers.calc_orientation_pixel(leftup,  FrontierMap.data, FrontierMap.info );
      collectedfrontiers=FloodfillFrontiers(leftup, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }


  if  ( rightup >= info_map.width -1 )  { 

    if (  (rightup_cell.get_identifier() == 5) || (rightup_cell.get_identifier() == 4) ||  
          (rightup_cell.get_identifier() == 7) || (rightup_cell.get_identifier() == 8)   ){

      //cout << "rechtsoben" << endl;
      collectedfrontiers.calc_orientation_pixel(rightup,  FrontierMap.data, FrontierMap.info );
      collectedfrontiers=FloodfillFrontiers(rightup, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }


  if  ( leftdown <= info_map.width * info_map.height-1 )  { 

    if (  (leftdown_cell.get_identifier() == 5) || (leftdown_cell.get_identifier() == 2) ||  
          (leftdown_cell.get_identifier() == 3) || (leftdown_cell.get_identifier() == 6)   ){

      //cout << "linksunten" << endl;
      collectedfrontiers.calc_orientation_pixel(leftdown,  FrontierMap.data, FrontierMap.info );
      collectedfrontiers=FloodfillFrontiers(leftdown, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }
  

  if ( rightdown <= info_map.width * info_map.height-1 )  {
    
    if (  (rightdown_cell.get_identifier() == 5) || (rightdown_cell.get_identifier() == 2) ||  
          (rightdown_cell.get_identifier() == 3) || (rightdown_cell.get_identifier() == 6)   ){

      //cout << "rechtsunten" << endl;
      collectedfrontiers.calc_orientation_pixel(rightdown,  FrontierMap.data, FrontierMap.info );
      collectedfrontiers=FloodfillFrontiers(rightdown, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }

  return collectedfrontiers; 
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
  //ros::Subscriber map_sub = nh.subscribe("map", 1, mapCallback);

  // Initialization for the ROS-Publisher to publish the new map data
  map_pub = nh.advertise<nav_msgs::OccupancyGrid>("frontier_exploration",1);

  while(ros::ok()){
    ros::spinOnce(); 
  }

  //  vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 1); 
  // while(ros::ok()){
  //   visualization_msgs::Marker marker;
  //   marker.header.frame_id = "map";
  //   marker.header.stamp = ros::Time();
  //   marker.ns = "my_namespace";
  //   marker.id = 0;
  //   marker.type = visualization_msgs::Marker::SPHERE_LIST;
  //   marker.action = visualization_msgs::Marker::ADD;
  //   marker.pose.orientation.x = 0.0;
  //   marker.pose.orientation.y = 0.0;
  //   marker.pose.orientation.z = 0.0;
  //   marker.pose.orientation.w = 1.0;
  //   marker.scale.x = 0.1;
  //   marker.scale.y = 0.1;
  //   marker.scale.z = 0.1;
  //   marker.color.a = 1.0; // Don't forget to set the alpha!
  //   marker.color.r = 0.0;
  //   marker.color.g = 1.0;
  //   marker.color.b = 0.0;
    
  //   for (int iter = 0; iter < every_frontier.size()  ; iter++){
  //     geometry_msgs::Point p;
  //     p.x = every_frontier[iter].get_gravity_of_center_x();
  //     p.y = every_frontier[iter].get_gravity_of_center_y();
  //     p.z = 0.0;
  //     marker.points.push_back(p);  
  //   }

  //   ros::spinOnce(); 
  //   vis_pub.publish( marker );
  //   }

  //ros::spin();

  return 0;
  
}

//Function for the received map data with the parameter nav_msgs --> OccupancyGrid
void mapCallback(const nav_msgs:: OccupancyGrid::ConstPtr& msg)
{ 
  tf::TransformListener listener;

  ros::Rate rate(10.0);
  tf::StampedTransform transform;
    try{
      listener.waitForTransform("/map", "/robot0", ros::Time(), ros::Duration(4.0));
      listener.lookupTransform( "/map" , "/robot0",ros::Time(0), transform);
      }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

  //Saving the parts of the OccupancyGrid in variables --> see documentation: doc.ros.org
  std_msgs::Header header_old = msg->header;
  nav_msgs::MapMetaData info_old = msg->info;
  std::vector<signed char> data_old = msg->data;
  std::vector <Frontiers> every_frontier;
  
  FrontierMap.header = header_old;
  FrontierMap.info = info_old;
  FrontierMap.data = data_old; 

  Frontiers addingfrontier;
  Pixel cell;
  int replacement = 10;
  int position = 0;
  
  
  //Show the height, width and size of the map
  ROS_INFO("Got map %d, %d", info_old.width, info_old.height);
  ROS_INFO_STREAM("size of map " << data_old.size());

  // 1. routine für abfrage der vier nachbarn schreiben
  // 2. schleife über alle zellen
  // 3. publisher für neue karte
  // 4. überlegen wie grenzen gefunden werden können (zellebene)
  for (int i =0; i < info_old.width *info_old.height; i++) {

    cell.check_pixeltype(i, data_old, info_old);

    //Pixel at the up-left corner

    if( cell.get_identifier() == 1){

      if (ExistedFrontier(i, every_frontier)==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center();
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        addingfrontier.calc_orientation_frontier();
        every_frontier.push_back(addingfrontier);
        addingfrontier.delete_frontier();
        replacement = replacement + 10;
      }

      continue;
    }
    

    //Pixel at the up-right corner

    if( cell.get_identifier() == 3){

      if (ExistedFrontier(i, every_frontier)==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center();
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        addingfrontier.calc_orientation_frontier();
        every_frontier.push_back(addingfrontier);
        addingfrontier.delete_frontier();
        replacement = replacement + 10;
      }
      
      continue;
    }
       
    //Pixels in the first line--> not included the up left/ up-right corner

    if( cell.get_identifier() == 2){

      if (ExistedFrontier(i, every_frontier)==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center();
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        addingfrontier.calc_orientation_frontier();
        every_frontier.push_back(addingfrontier);
        replacement = replacement + 10;
        addingfrontier.delete_frontier();
      }
      
      continue;
    }

    // Pixels in the first column --> not included up-left / down left corner

    if( cell.get_identifier() == 4){

      if (ExistedFrontier(i, every_frontier)==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center();
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        addingfrontier.calc_orientation_frontier();
        every_frontier.push_back(addingfrontier);
        replacement = replacement + 10;
        addingfrontier.delete_frontier();
      }
      
      continue;
      
    }
    
    // Pixel at the down-left corner

    if(cell.get_identifier() == 7){

      if (ExistedFrontier(i, every_frontier)==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center();
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        addingfrontier.calc_orientation_frontier();
        every_frontier.push_back(addingfrontier);
        replacement = replacement + 10;
        addingfrontier.delete_frontier();
      }
    
      continue;
    }

    // Pixels in the last line--> not included the down-left / down right corner

    if( cell.get_identifier() == 8 ){

      if (ExistedFrontier(i, every_frontier)==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center();
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        addingfrontier.calc_orientation_frontier();
        every_frontier.push_back(addingfrontier);
        replacement = replacement + 10;
        addingfrontier.delete_frontier();
      }
      
      continue;
    }

    // Pixels in the last column --> not included the up-right / down right corner

    if ( cell.get_identifier() == 6  ){

      if (ExistedFrontier(i, every_frontier)==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center();
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        addingfrontier.calc_orientation_frontier();
        every_frontier.push_back(addingfrontier);
        replacement = replacement + 10;
        addingfrontier.delete_frontier();
      }

      continue;
    }

    //Pixel at the down-right corner

    if( cell.get_identifier() == 9 ){

      if (ExistedFrontier(i, every_frontier)==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center();
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        addingfrontier.calc_orientation_frontier();
        every_frontier.push_back(addingfrontier);
        replacement = replacement + 10;
        addingfrontier.delete_frontier();
      }

      continue;
    }

    //Other Pixels

    if ( cell.get_identifier() == 5 ){

      if (ExistedFrontier(i, every_frontier)==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center();
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        addingfrontier.calc_orientation_frontier();
        every_frontier.push_back(addingfrontier);
        addingfrontier.delete_frontier();
        replacement = replacement + 10;
      }
    }
    cell.delete_pixel();
  }
 
  cout << every_frontier.size() << endl;
  std::cout << FrontierMap.data.size() << std::endl; 
  
  // Initialization for the ROS-Publisher to publish the new map data
  map_pub.publish(FrontierMap);
  ROS_INFO_STREAM("Publishing the NewMap.\n" );
  position = BiggestFrontier( every_frontier);
  //position = ShortestDistanceFrontier( every_frontier);
  MoveBase next_target( every_frontier[position].get_gravity_of_center_x(), every_frontier[position].get_gravity_of_center_y(), 0, every_frontier[position].get_orientation_frontier_y(), 0, 0, every_frontier[position].get_orientation_frontier_x());
  every_frontier.clear();
}



