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

//Including Navigation Messages --> OccupancyGrid
#include <nav_msgs/OccupancyGrid.h>

//Including Geometrry Messages --> Twist
#include <geometry_msgs/Twist.h>

// Creating a convenience typedef for a SimpleActionClient; For communication with the MoveBaseAction action interface.
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


using namespace std;

// class Frontiers for groupping the Frontiers
class Frontiers {

  // private attributs
  private:

    // standardvector for postions of the frontierpixel
    std:: vector <int> pixels;
    // standardvector for transformation from pixel to x and y-coordinates   
    std:: vector <float> xpos;
    std:: vector <float> ypos;
    // standardvector from datatype 3-D Vector(x,y,z) for orientation
    std:: vector <tf::Vector3> orientation_pixels;
    // contains the number of the pixel in a frontier
    int number_of_pixel;
    // gravity of center x and y
    float gravity_of_center_x;
    float gravity_of_center_y;
    // euclidian distance 
    float distance;
    // orientation of the frontier x and y
    float orientation_frontier_x;
    float orientation_frontier_y;

  // public methods
  public:

    // constructor: set attributes to zero
    Frontiers(){
      number_of_pixel = 0;
      gravity_of_center_x = 0;
      gravity_of_center_y = 0;
      distance = 0;
      orientation_frontier_x = 0;
      orientation_frontier_y = 0;
    }
    
    // method for deleting the frontier
    // delete standardvector and set attributes to zeros
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

    // Getter for the attributes

    int get_pixel_size(){
      number_of_pixel = pixels.size();
      return number_of_pixel;
    }

    // Getter for position in the standardvector pixels
    int get_pixel(int index){
      return pixels[index];
    }

    float get_gravity_of_center_x(){
      return gravity_of_center_x;
    }

   float get_gravity_of_center_y(){
      return gravity_of_center_y;
    }

    float get_distance(){
      return distance;
    }

    float get_orientation_frontier_x(){
      return orientation_frontier_x;
    }

    float get_orientation_frontier_y(){
      return orientation_frontier_y;
    }

    //pushing back the argument to the standardvector pixels 
    void set_pixels(int pos){
      pixels.push_back(pos);
    }

    // method for printing attributes in the terminal for debugging
    void print_frontier(){

      // Loop for run through the standardvector pixels
      for (int i = 0; i < pixels.size(); i++){
        
        // printing the attributes
        cout<< "Pixel in the Frontiers are: "<< pixels[i] <<endl;
        cout<< "Pixel in x coordinate: "<< xpos[i] <<endl;
        cout<< "Pixel in y_coordinate: "<< ypos[i] <<endl;
      }
        cout<< "gravity_of_center x coordinate: "<< gravity_of_center_x <<endl;
        cout<< "gravity_of_center y_coordinate: "<< gravity_of_center_y <<endl;
    }
    
    // checking, if the pixel is already in the Frontier  
    bool pixel_item_exists(int number){

      // Loop for run through the standardvector pixels
      for (int j = 0; j < pixels.size(); j++){

        if(pixels[j]==number){  
          return true;
        }
      }
      return false;
    }

  // method for transformation from pixel to x and y-coordinates 
  void pixels_x_y_transformation (nav_msgs::MapMetaData info_x){
    // variables for calculation x and y coordinates
    float x = 0;
    float y = 0;

    // Loop for run through the standardvector pixels
    for (int j = 0; j < pixels.size(); j++){

      // Calculation of the x-coordinate:
      // 1. Divide (modulo) the current pixel with width of the map
      // 2. Multiply the map resolution
      // 3. Add the x-coordinate from the origin of the map
      x = pixels[j] % info_x.width * info_x.resolution + info_x.origin.position.x;
      
      // Calculation of the y-coordinate:
      // 1. Divide the current pixel with width of the map
      // 2. Multiply the map resolution
      // 3. Add the y-coordinate from the origin of the map
      y = pixels[j]  / info_x.width * info_x.resolution + info_x.origin.position.y;
      
      // Pushing back the coordinates to the standardvector 
      xpos.push_back(x);
      ypos.push_back(y);

    }
  }

  // method for calculation the gravity of center 
  void calc_gravity_of_center (std::vector<signed char> data_move_base, nav_msgs::MapMetaData info_move_base ){
    
    // variables for adding all x and y coordinates
    float x = 0;
    float y = 0;

    // variables for back transformation form coordinates to pixels 
    int pixel = 0;
    int pixel_x = 0;
    int pixel_y = 0;

    // variables for possible status of the pixels
    int occupied_pixel = 100;
    int free_pixel = 0;
    int unexplored_pixel = -1;

    // Loop for run through the standardvector xpos
    for (int j = 0; j < xpos.size(); j++){
      
      // adding all x-coordinates and y-coordinates
      x = x + xpos[j];
      y = y + ypos[j];
    }

    // calculation of the gravity of center
    // divide all coordinates with size of the standardvector
    gravity_of_center_x = x/xpos.size();
    gravity_of_center_y = y/ypos.size();

    // back transfromation form coordinates to pixel
    // opposite form method pixels_x_y_transformation
    pixel_x = ((gravity_of_center_x - info_move_base.origin.position.x) / info_move_base.resolution) ;
    pixel_y = ((gravity_of_center_y - info_move_base.origin.position.y) / info_move_base.resolution) * info_move_base.width;
    pixel = pixel_x + pixel_y;

    // if gravity of center is an occupied pixel 
    if (data_move_base[pixel] == occupied_pixel){

      // Loop for run through the standardvector pixels
      for (int k = 0; k <pixels.size(); k++){
        
        // select the first pixel as the new gravitiy of center, which is free 
        if ( (data_move_base[pixels[k]] == free_pixel ) ){
          gravity_of_center_x = xpos[k];
          gravity_of_center_y = ypos[k];
          break;
        } 
      }
    }
      
  }

  // calculation of the euclidian distance (robot postion and gravity of center)
  void euclidean_distance (float current_x, float current_y, float future_x , float future_y){
    distance = sqrt( (current_x-future_x) * (current_x-future_x) + (current_y - future_y) * (current_y - future_y) ) ;

  } 

  // calculation for orientation of a Frontier
  void calc_orientation_pixel(int latest, std::vector<signed char> data_orientation, nav_msgs::MapMetaData info_orientation )
  { 
    // 3-D vector for calculation of the oreiantation
    tf::Vector3 latest_orientation (0,0,0);
    // variables for status of an unexplored
    int unexplored_pixel = -1;
    
    // For each all 8 possibilties 

    // Right
    // Index may not in the last column
    // add 1 to the x-coordinate
    if( (latest % info_orientation.width != info_orientation.width -1) && (data_orientation[latest+1]== unexplored_pixel) ){
      latest_orientation = latest_orientation + tf::Vector3 (1,0,0);
    }

    // Left
    // Index may not in the first column
    // subtract 1 to the x-coordinate
    if( (latest % info_orientation.width != 0) && (data_orientation[latest-1]== unexplored_pixel) ){
      latest_orientation = latest_orientation + tf::Vector3 (-1,0,0);
    }

    // Up 
    // Index may not in the first line
    // subtract 1 to the y-coordinate  
    if( (latest>= info_orientation.width -1) && (data_orientation[latest-info_orientation.width]== unexplored_pixel) ){
      latest_orientation = latest_orientation + tf::Vector3 (0,-1,0);
    }

    // Down
    // Index may not in the last line
    // add 1 to the y-coordinate
    if( (latest<= info_orientation.width * info_orientation.height-1) && (data_orientation[latest+info_orientation.width]== unexplored_pixel) ){
      latest_orientation = latest_orientation + tf::Vector3 (0,1,0);
    }

    // LeftUp
    // Index may not in the first line and column
    // subtract 1 to the x and y-coordinate
    if( (latest >= info_orientation.width -1) && (latest % info_orientation.width != 0) && (data_orientation[latest-info_orientation.width-1]== unexplored_pixel) ){
      latest_orientation = latest_orientation + tf::Vector3 (-1,-1,0);
    }

    // RightUp
    // Index may not in the first line and last column
    // add 1 to the x-coordinate and subtract 1 y-coordinate
    if( (latest >= info_orientation.width -1) && (latest % info_orientation.width != info_orientation.width -1) && (data_orientation[latest-info_orientation.width+1]== unexplored_pixel) ){
      latest_orientation = latest_orientation + tf::Vector3 (1,-1,0);
    }

    // LeftDown
    // Index may not in the last line and first column
    // subtract 1 to the x-coordinate and add 1 y-coordinate
    if( (latest <= info_orientation.width * info_orientation.height-1) && (latest % info_orientation.width != 0) && (data_orientation[latest+info_orientation.width-1]== unexplored_pixel) ){
      latest_orientation = latest_orientation + tf::Vector3 (-1,1,0);
    }

    // RightDown
    // Index may not in the last line and  column
    // add 1 to the x-coordinate and y-coordinate
    if( (latest <= info_orientation.width * info_orientation.height-1) && (latest % info_orientation.width != info_orientation.width -1)&& (data_orientation[latest+info_orientation.width+1]== unexplored_pixel) ){
      latest_orientation = latest_orientation + tf::Vector3 (1,1,0);
    }
    
    // pushing back the orientation in the standardvector
    orientation_pixels.push_back(latest_orientation);
  }

  // calculation of the orientation of the Frontier
  void calc_orientation_frontier(){

    // variables for the adding the x and y orientation 
    float sum_x = 0;
    float sum_y = 0;

    // Loop for run through the standardvector orientation_pixels 
    for (int j = 0; j < orientation_pixels.size(); j++){
      
      // adding all x and y orientation 
      sum_x = sum_x + orientation_pixels[j][0];
      sum_y = sum_y + orientation_pixels[j][1];
    }

    // divide the sum of the x and y orienatation 
    // with the size of the standardvector orientation_pixels
    sum_x = sum_x/ orientation_pixels.size();
    sum_y = sum_y/ orientation_pixels.size();

    // saving the variables in the attributes  
    orientation_frontier_x = sum_x;
    orientation_frontier_y = sum_y;
  }

};


// class for checking the pixeltype 
class Pixel {
   
  // private attributes
  private:

  // attribut defines the pixeltype:
  // 1 : Up left corner
  // 2 : First line without the up left and right corner 
  // 3 : Up right corner
  // 4 : First column without up and down left corner
  // 6 : Last column without up and down right corner
  // 7 : Down left corner
  // 8 : Last line without the down left and right corner 
  // 9 : Down right corner
  // 5 : Center pixeltype 
  int identifier;

  // public methods
  public:

    // constructor: Set attribut zero
    Pixel(){
      identifier = 0;
    }

    // method for checking the pixeltype
    void check_pixeltype(int currentpos, std::vector<signed char> data_current, nav_msgs::MapMetaData info_current){
      
      // variables for status of a pixel
      int free_pixel = 0;
      int unexplored_pixel = -1;

      // Checking if it is the up left corner
      // Checking the neighbours for a frontier 
      if(currentpos == 0){

        if( (data_current[currentpos] ==free_pixel) && ( (data_current[currentpos+1] == unexplored_pixel) || (data_current[currentpos+info_current.width] == unexplored_pixel) ) ){
          identifier = 1;
        }
      }

      // Checking if it is the first line
      // Checking the neighbours for a frontier
      else if( (currentpos<info_current.width-1)&&(currentpos!=0) ){

        if( (data_current[currentpos] == free_pixel) && ((data_current[currentpos-1] ==unexplored_pixel) || (data_current[currentpos+1] ==unexplored_pixel) || (data_current[currentpos+info_current.width] ==unexplored_pixel) ) ){
          identifier = 2;
        }
      }

      // Checking if it is the up right corner
      // Checking the neighbours for a frontier
      else if(currentpos == info_current.width-1){

        if( (data_current[currentpos] == free_pixel) && ((data_current[currentpos-1] == unexplored_pixel) || (data_current[currentpos+info_current.width] ==unexplored_pixel) ) ){
          identifier = 3;
        }
      }
      
      // Checking if it is the first column
      // Checking the neighbours for a frontier
      else if( (currentpos % info_current.width == 0) && (currentpos!=0) && (currentpos != info_current.width * (info_current.height-1) ) ){

        if  ( (data_current[currentpos] ==free_pixel) &&( (data_current[currentpos-info_current.width] ==unexplored_pixel) || (data_current[currentpos+1] == unexplored_pixel) || (data_current[currentpos+info_current.width] == unexplored_pixel))){
          identifier = 4;
        }
      }

      // Checking if it is the last column
      // Checking the neighbours for a frontier
      else if( (currentpos % info_current.width == info_current.width -1) && (currentpos!=info_current.width-1) && (currentpos != info_current.width * info_current.height -1)){

        if( (data_current[currentpos] ==free_pixel) && ((data_current[currentpos-info_current.width] ==unexplored_pixel) || (data_current[currentpos-1] ==unexplored_pixel) ||  (data_current[currentpos+1] ==unexplored_pixel) )){
          identifier = 6;
        }
      }

      // Checking if it is the down left corner
      // Checking the neighbours for a frontier
      else if(currentpos == info_current.width * (info_current.height-1)){
    
        if( (data_current[currentpos] == free_pixel) && ( (data_current[currentpos-info_current.width] == unexplored_pixel) || (data_current[currentpos+1] == unexplored_pixel))){
          identifier = 7;
        }
      }

      // Checking if it is the last line
      // Checking the neighbours for a frontier
      else if( (currentpos >info_current.width *(info_current.height-1)) && (currentpos< info_current.width * info_current.height-1) ){

        if( (data_current[currentpos] ==free_pixel) && ((data_current[currentpos-1] ==unexplored_pixel) || (data_current[currentpos-info_current.width] ==unexplored_pixel) || (data_current[currentpos+1] ==unexplored_pixel) ) ){
          identifier = 8;
        }
      }

      // Checking if it is the down right corner
      // Checking the neighbours for a frontier
      else if (currentpos == info_current.width * info_current.height-1 ){

        if( (data_current[currentpos] == free_pixel) && ((data_current[currentpos-1] == unexplored_pixel) || (data_current[currentpos-info_current.width] == unexplored_pixel) ) ){
          identifier = 9;
        }
      }
      
      // Checking if it is a center pixel
      // Checking the neighbours for a frontier
      else {
        if( (data_current[currentpos]==free_pixel)  && ( (data_current[currentpos-1] == unexplored_pixel) || 
        (data_current[currentpos+1] == unexplored_pixel) || (data_current[currentpos - info_current.width]==unexplored_pixel)|| 
        (data_current[currentpos + info_current.width]==unexplored_pixel)  ) )
        {

          identifier = 5;
        }
      }

    }

    // Set attribute to back to zero
    void delete_pixel(){
      identifier = 0;
    }
    
    // Getter
    int get_identifier(){
      return identifier;
    }
};

// class for sending coordinates 
// and oreintation to the move base server 
class MoveBase{

  // private attributes
  private:
    // MoveBaseGoal.msg: 
    // defines a target coordinate and orienatation
    move_base_msgs::MoveBaseGoal goal;

  // public methods:
  public:

    // constructor:
    // For sending target pose and orientattion
    MoveBase( float position_x, float position_y, float position_z, float orientation_w, float orientation_x, float orientation_y, float orientation_z ){
    
      // Name of the frame, which belongs 
      // to the target pose and orientation 
      goal.target_pose.header.frame_id = "map";
      
      // current time
      goal.target_pose.header.stamp = ros::Time::now();
      
      // define the x-, y- and z-coordinates for the the target pose
      goal.target_pose.pose.position.x = position_x; 
      goal.target_pose.pose.position.y = position_y;
      goal.target_pose.pose.position.z = position_z;

      // define the quaternion for the orientation
      goal.target_pose.pose.orientation.w = orientation_w;
      goal.target_pose.pose.orientation.x = orientation_x;
      goal.target_pose.pose.orientation.y = orientation_y;
      goal.target_pose.pose.orientation.z = orientation_z;

      // tell the action client to spin a thread by default
      MoveBaseClient ac("move_base", true);

      // wait for the action server to come up
      while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
      }

      // Send the target position and wait for it
      ROS_INFO("Sending goal");
      ac.sendGoal(goal);
      ac.waitForResult();

      // check if the movement of the robot was successfull
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("The robot moved successfully to target postion !");
      else
        ROS_INFO("The robot failed to move for some reason");

    }

};

// global variables: Publisher and the Frontiermap
ros::Publisher map_pub;
nav_msgs:: OccupancyGrid FrontierMap;

// Function for the received map data with the parameter nav_msgs --> OccupancyGrid
void mapCallback(const nav_msgs:: OccupancyGrid::ConstPtr& msg);

// Function for checking if a Frontier already exists
bool ExistedFrontier (int position, std::vector <Frontiers> every_frontier){

  // Loop for run through the standardvector every_frontier
  for (int iter = 0; iter < every_frontier.size(); iter++){
    
    // Check if a frontierpixel is already in a Frontier
    if( every_frontier[iter].pixel_item_exists(position)== true){
      return true;
    } 
     
  }
  return false;
}

// Function for finding the Frontier with the most frontierpixels
int BiggestFrontier (std::vector <Frontiers> every_frontier){

  // variables for remember the size 
  // and the position in the standardvector
  int size_of_frontier = 0;
  int frontier_pos = 0;

  // Loop for run through the standardvector every_frontier
  for (int iter = 0; iter < every_frontier.size(); iter++){

    // Checking if the current size is lower
    if( every_frontier[iter].get_pixel_size() > size_of_frontier){
      size_of_frontier = every_frontier[iter].get_pixel_size();
      frontier_pos = iter;
    } 
  }
    return frontier_pos;
}

// Function for finding the Frontier with the shortest distance
// to the current position of the robot 
int ShortestDistanceFrontier (std::vector <Frontiers> every_frontier){

  // variables for remember the shortest distance
  // and the position in the standardvector
  int distance_of_frontier = every_frontier[0].get_distance();
  int pos_frontier = 0;

  // Loop for run through the standardvector every_frontier
  for (int iter = 0; iter < every_frontier.size(); iter++){

    // Checking if the current distance is longer
    if( every_frontier[iter].get_distance() < distance_of_frontier)
    { 
      distance_of_frontier=every_frontier[iter].get_distance();
      pos_frontier = iter;
    } 
  }
    return pos_frontier;
}

// Function for the floodfill algorithm
Frontiers FloodfillFrontiers(int counter, int substitution, std::vector<signed char> data_map, nav_msgs::MapMetaData info_map,  Frontiers transfer ){
  
  // // variables for status of a pixel 
  int free_pixel = 0;
  int unexplored_pixel = -1;

  // Calculation of the 8 possible neighbours and check pixeltype:

  // right neighbour: add 1 to current pixel
  int right = counter + 1;
  Pixel right_cell;
  right_cell.check_pixeltype(right, FrontierMap.data, FrontierMap.info);

  // left neighbour: subtract 1 from current pixel 
  int left = counter - 1;
  Pixel left_cell ;
  left_cell.check_pixeltype(left, FrontierMap.data, FrontierMap.info);

  // up neighbour: add the width of the map to current pixel 
  int up = counter - info_map.width;
  Pixel up_cell ;
  up_cell.check_pixeltype(up, FrontierMap.data, FrontierMap.info);

  // down neighbour: subtract the width of the map from current pixel 
  int down = counter + info_map.width;
  Pixel down_cell ;
  down_cell.check_pixeltype(down, FrontierMap.data, FrontierMap.info);

  // left up neighbour: subtract the width of the map and 1 from current pixel
  int leftup = counter - info_map.width - 1;
  Pixel leftup_cell ;
  leftup_cell.check_pixeltype(leftup, FrontierMap.data, FrontierMap.info);

  // right up neighbour: subtract the width of the map and add 1 from current pixel
  int rightup = counter  - info_map.width + 1;
  Pixel rightup_cell ;
  rightup_cell.check_pixeltype(rightup, FrontierMap.data, FrontierMap.info);

  // left down neighbour: add the width of the map and subtract 1 to current pixel
  int leftdown = counter + info_map.width - 1;
  Pixel leftdown_cell ;
  leftdown_cell.check_pixeltype(leftdown, FrontierMap.data, FrontierMap.info);

  // right down neighbour: add the width of the map and 1 to current pixel
  int rightdown = counter  + info_map.width + 1;
  Pixel rightdown_cell ;
  rightdown_cell.check_pixeltype(rightdown, FrontierMap.data, FrontierMap.info);
  
  // create a frontier object 
  // add the current pixel to the frontier
  Frontiers collectedfrontiers = transfer;
  collectedfrontiers.set_pixels(counter);

  // if the needed colour already is written in the current pixel data
  if (FrontierMap.data[counter] == substitution) {
    return collectedfrontiers;
  }

  // if the needed colour is not written in the current pixel
  else if  ( (FrontierMap.data[counter] != free_pixel ) && (FrontierMap.data[counter] != unexplored_pixel ) ){
    return collectedfrontiers;
  }
  
  // write the needed colour in the current pixel
  else { 
    FrontierMap.data[counter] = substitution;
   }
  
  // check if the right neighbour is in the same line
  if  ( (right % info_map.width != info_map.width -1)  ) { 
    
    // only these pixeltypes have an right neighbour
    if (  (right_cell.get_identifier() == 5) || (right_cell.get_identifier() == 1) || (right_cell.get_identifier() == 2) || 
          (right_cell.get_identifier() == 4) || (right_cell.get_identifier() == 7) || (right_cell.get_identifier() == 8)   ){
      
      // call the method for calculation of the orientation of the Frontier
      collectedfrontiers.calc_orientation_pixel(right,  FrontierMap.data, FrontierMap.info );
      // recursive functioncall floodfill algorithm
      collectedfrontiers=FloodfillFrontiers(right, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }
  
  // check if the left neighbour is in the same line
  if  ( left % info_map.width != 0 ) { 
    
    // only these pixeltypes have an left neighbour
    if (  (left_cell.get_identifier() == 5) || (left_cell.get_identifier() == 2) || (left_cell.get_identifier() == 3) || 
          (left_cell.get_identifier() == 6) || (left_cell.get_identifier() == 8) || (left_cell.get_identifier() == 9)   ){

      collectedfrontiers.calc_orientation_pixel(left,  FrontierMap.data, FrontierMap.info );
      collectedfrontiers=FloodfillFrontiers(left, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }
  
  // check if the up neighbour is not out of boundary 
  if  ( up >= info_map.width -1 )   {
    
    // only these pixeltypes have an up neighbour
    if (  (up_cell.get_identifier() == 5) || (up_cell.get_identifier() == 4) || (up_cell.get_identifier() == 6) || 
          (up_cell.get_identifier() == 7) || (up_cell.get_identifier() == 8) || (up_cell.get_identifier() == 9)   ){


      collectedfrontiers.calc_orientation_pixel(up,  FrontierMap.data, FrontierMap.info );
      collectedfrontiers=FloodfillFrontiers(up, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }
  
  // check if the down neighbour is not out of boundary 
  if   ( down <= info_map.width * info_map.height-1 )  { 
    
    // only these pixeltypes have an down neighbour
    if (  (down_cell.get_identifier() == 5) || (down_cell.get_identifier() == 1) || (down_cell.get_identifier() == 2) || 
          (down_cell.get_identifier() == 3) || (down_cell.get_identifier() == 4) || (down_cell.get_identifier() == 6)   ){

      collectedfrontiers.calc_orientation_pixel(down,  FrontierMap.data, FrontierMap.info );
      collectedfrontiers=FloodfillFrontiers(down, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }
 
  // check if the left up neighbour is not out of boundary 
  if  ( leftup >= info_map.width -1 )  { 

    // only these pixeltypes have an left up neighbour
    if (  (leftup_cell.get_identifier() == 5) || (leftup_cell.get_identifier() == 6) ||  
          (leftup_cell.get_identifier() == 8) || (leftup_cell.get_identifier() == 9)   ){

      collectedfrontiers.calc_orientation_pixel(leftup,  FrontierMap.data, FrontierMap.info );
      collectedfrontiers=FloodfillFrontiers(leftup, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }

  // check if the right up neighbour is not out of boundary 
  if  ( rightup >= info_map.width -1 )  { 

    // only these pixeltypes have an right up neighbour
    if (  (rightup_cell.get_identifier() == 5) || (rightup_cell.get_identifier() == 4) ||  
          (rightup_cell.get_identifier() == 7) || (rightup_cell.get_identifier() == 8)   ){

      collectedfrontiers.calc_orientation_pixel(rightup,  FrontierMap.data, FrontierMap.info );
      collectedfrontiers=FloodfillFrontiers(rightup, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }

  // check if the left down neighbour is not out of boundary 
  if  ( leftdown <= info_map.width * info_map.height-1 )  { 

    // only these pixeltypes have an left down neighbour
    if (  (leftdown_cell.get_identifier() == 5) || (leftdown_cell.get_identifier() == 2) ||  
          (leftdown_cell.get_identifier() == 3) || (leftdown_cell.get_identifier() == 6)   ){

      collectedfrontiers.calc_orientation_pixel(leftdown,  FrontierMap.data, FrontierMap.info );
      collectedfrontiers=FloodfillFrontiers(leftdown, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }
  
   // check if the right down neighbour is not out of boundary 
  if ( rightdown <= info_map.width * info_map.height-1 )  {
    
    // only these pixeltypes have an right down neighbour
    if (  (rightdown_cell.get_identifier() == 5) || (rightdown_cell.get_identifier() == 2) ||  
          (rightdown_cell.get_identifier() == 3) || (rightdown_cell.get_identifier() == 6)   ){

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

int main( int argc, char ** argv){

  // Initialization of ros node
  ros::init(argc, argv , "thi_exploration");

  // Initialization of node handle for communication with ROSCORE
  ros::NodeHandle nh;

  // Initialization for the ROS-Subscriber to subscribe the map data:

  // subscribe the map from gmapping
  ros::Subscriber map_sub = nh.subscribe("gmapping/map", 1, mapCallback);

  // subscribe the map from hector slam 
  // ros::Subscriber map_sub = nh.subscribe("map_from_gmapping", 1, mapCallback);

  // Initialization for the ROS-Publisher to publish the new map data:
  // publisher for the Frontier map
  map_pub = nh.advertise<nav_msgs::OccupancyGrid>("frontier_exploration",1);
  
  // Spin the ros node until the nodes are running correctly
  while(ros::ok()){
    ros::spinOnce(); 
  }

  return 0;
  
}

// Function for the received map data with the parameter nav_msgs --> OccupancyGrid
void mapCallback(const nav_msgs:: OccupancyGrid::ConstPtr& msg){

  // needed variables for creating a tf transformation tree
  // get the current postion from the robot 
  // for calculation the distance from a frontier
  tf::TransformListener listener;
  ros::Rate rate(10.0);
  tf::StampedTransform transform;
  
  try{
      // creating a tf transformation tree for the simulation 
      listener.waitForTransform("/map", "/robot0", ros::Time(), ros::Duration(4.0));
      listener.lookupTransform( "/map" , "/robot0",ros::Time(0), transform);

      // creating a tf transformation tree for the robot Pioneer 3-AT 
      // listener.waitForTransform("/map", "/RosAria/pose", ros::Time(), ros::Duration(4.0));
      // listener.lookupTransform( "/map" , "/RosAria/pose",ros::Time(0), transform);
    }
  catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
  }

  // Saving the parts of the OccupancyGrid in variables
  std_msgs::Header header_old = msg->header;
  nav_msgs::MapMetaData info_old = msg->info;
  std::vector<signed char> data_old = msg->data;

  // standardvector for all frontiers
  std::vector <Frontiers> every_frontier;
  
  // write the data from the received map to the frontier map
  FrontierMap.header = header_old;
  FrontierMap.info = info_old;
  FrontierMap.data = data_old; 

  // creating a frontierobject and a pixeltype
  Frontiers addingfrontier;
  Pixel cell;
  
  // number of the colour for the floodfill algorithm
  int replacement = 10;
  // variable for remembering the position of the frontier in the standardvector every_frontier
  int position = 0;
  
  // Show the height, width and size of the map
  ROS_INFO("Got map %d, %d", info_old.width, info_old.height);
  ROS_INFO_STREAM("size of map " << data_old.size());

  // For every 9 of the pixeltype will be executed the follow staps:
  // check if the pixel already is in a frontier
  // call the floodfill algorithm
  // transformate the pixel to the x- and y-coordinate
  // calculate the gravity of center
  // calculate the euclidean distance between the robot and a frontier
  // calculate the orientation of the frontier
  // push back the current frontier to the standardvector every_frontier
  // delete the current frontier
  // add 10 to the current colour
  // continue the loop

  // Loop for running through every pixel in the received map
  for (int i = 0; i < info_old.width *info_old.height; i++) {

    // check the current pixel for its pixeltype
    cell.check_pixeltype(i, data_old, info_old);

    // check if the current pixel is the up-left corner
    if( cell.get_identifier() == 1){
      
      if (ExistedFrontier(i, every_frontier)==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center(FrontierMap.data, FrontierMap.info);
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        addingfrontier.calc_orientation_frontier();
        every_frontier.push_back(addingfrontier);
        addingfrontier.delete_frontier();
        replacement = replacement + 10;
      }
      continue;
    }
    
    // check if the current pixel is the up-right corner
    if( cell.get_identifier() == 3){

      if (ExistedFrontier(i, every_frontier)==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center(FrontierMap.data, FrontierMap.info);
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        addingfrontier.calc_orientation_frontier();
        every_frontier.push_back(addingfrontier);
        addingfrontier.delete_frontier();
        replacement = replacement + 10;
      }
      continue;
    }
       
    // check if the current pixel is in the first line--> not included the up left/ up-right corner
    if( cell.get_identifier() == 2){

      if (ExistedFrontier(i, every_frontier)==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center(FrontierMap.data, FrontierMap.info);
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        addingfrontier.calc_orientation_frontier();
        every_frontier.push_back(addingfrontier);
        replacement = replacement + 10;
        addingfrontier.delete_frontier();
      }
      
      continue;
    }

    // check if the current pixel is in the first column --> not included up-left / down left corner
    if( cell.get_identifier() == 4){

      if (ExistedFrontier(i, every_frontier)==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center(FrontierMap.data, FrontierMap.info);
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        addingfrontier.calc_orientation_frontier();
        every_frontier.push_back(addingfrontier);
        replacement = replacement + 10;
        addingfrontier.delete_frontier();
      }
      
      continue;
      
    }
    
    // check if the current pixel is the down-left corner
    if(cell.get_identifier() == 7){

      if (ExistedFrontier(i, every_frontier)==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center(FrontierMap.data, FrontierMap.info);
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        addingfrontier.calc_orientation_frontier();
        every_frontier.push_back(addingfrontier);
        replacement = replacement + 10;
        addingfrontier.delete_frontier();
      }
      continue;
    }

    // check if the current pixel is in the last line--> not included the down-left / down right corner
    if( cell.get_identifier() == 8 ){

      if (ExistedFrontier(i, every_frontier)==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center(FrontierMap.data, FrontierMap.info);
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        addingfrontier.calc_orientation_frontier();
        every_frontier.push_back(addingfrontier);
        replacement = replacement + 10;
        addingfrontier.delete_frontier();
      }
      continue;
    }

    // check if the current pixel is in the last column --> not included the up-right / down right corner
    if ( cell.get_identifier() == 6  ){

      if (ExistedFrontier(i, every_frontier)==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center(FrontierMap.data, FrontierMap.info);
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        addingfrontier.calc_orientation_frontier();
        every_frontier.push_back(addingfrontier);
        replacement = replacement + 10;
        addingfrontier.delete_frontier();
      }
      continue;
    }

    // check if the current pixel is the down-right corner

    if( cell.get_identifier() == 9 ){

      if (ExistedFrontier(i, every_frontier)==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center(FrontierMap.data, FrontierMap.info);
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        addingfrontier.calc_orientation_frontier();
        every_frontier.push_back(addingfrontier);
        replacement = replacement + 10;
        addingfrontier.delete_frontier();
      }
      continue;
    }

    // check if the current pixel is a center pixel
    if ( cell.get_identifier() == 5 ){

      if (ExistedFrontier(i, every_frontier)==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center(FrontierMap.data, FrontierMap.info);
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        addingfrontier.calc_orientation_frontier();
        every_frontier.push_back(addingfrontier);
        addingfrontier.delete_frontier();
        replacement = replacement + 10;
      }
    }

    // delete the current pixel object
    cell.delete_pixel();
  }

  // print the number of the frontiers
  cout << every_frontier.size() << endl;
  // print the size of the Frontier map
  std::cout << FrontierMap.data.size() << std::endl; 
  
  // publish the Frontier map 
  map_pub.publish(FrontierMap);
  ROS_INFO_STREAM("Publishing the NewMap.\n" );

  // look for the Frontier with the most frontierpixels
  // position = BiggestFrontier( every_frontier);

  // look for the Frontier with the shortest distance to the current robot position 
  position = ShortestDistanceFrontier( every_frontier);

  // send the position and the orientation of the selected frontier to the move base client
  MoveBase next_target( every_frontier[position].get_gravity_of_center_x(), every_frontier[position].get_gravity_of_center_y(), 0, 1, 0, 0, every_frontier[position].get_orientation_frontier_x()+ every_frontier[position].get_orientation_frontier_y());
  
  // delete the standardvector every_frontier  
  every_frontier.clear();
}



