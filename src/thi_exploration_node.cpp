// Including the input and output stream
#include <iostream>
#include <vector>
// Including the ROS libary
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

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
    int number_of_pixel;
    int number_of_diagonals;
    float gravity_of_center_x;
    float gravity_of_center_y;
    float distance;

  public:

    Frontiers(){
      //cout << "object created" <<endl;
      number_of_pixel = 0;
      number_of_diagonals = 0;
      gravity_of_center_x = 0;
      gravity_of_center_y = 0;
      distance = 0;

    }

    void delete_pixels(){
      pixels.clear();
      xpos.clear();
      ypos.clear();
      number_of_pixel = 0;
      number_of_diagonals = 0;
      gravity_of_center_x = 0;
      gravity_of_center_y = 0;
      distance = 0;
    }
    
    void set_pixels( int pos){
      pixels.push_back(pos);

    }

    void print_pixels(){
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
          //cout<< "Pixel is in Frontiers " << endl;
          return true;
        }
      }
      //cout<< "Pixel is not in Frontiers " << endl;
      return false;
    }

    int get_pixel_size(){
      number_of_pixel = pixels.size();
      return number_of_pixel;
    }

    int get_diagonals(){
      return number_of_diagonals;
    }

    void diagonal_registered(){
      number_of_diagonals = number_of_diagonals + 1;
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

  void pixels_x_y_transformation (nav_msgs::MapMetaData info_x){
    float x = 0;
    float y = 0;
    for (int j = 0; j < pixels.size(); j++){
      x = pixels[j] % info_x.width * info_x.resolution + info_x.origin.position.x;
      y =  pixels[j]  / info_x.width * info_x.resolution + info_x.origin.position.y;
      xpos.push_back(x);
      ypos.push_back(y);
      // cout<< "Pixel in x coordinate: "<< xpos[j] <<endl;
      // cout<< "Pixel in y_coordinate: "<< ypos[j] <<endl;

    }
  }

   void calc_gravity_of_center (){
    float x = 0;
    float y = 0;
    for (int j = 0; j < pixels.size(); j++){
      x = x + xpos[j];
      y = y + ypos[j];
    }
    gravity_of_center_x = x/xpos.size() - 0.5;
    gravity_of_center_y = y/xpos.size() - 0.5;
  }

  void euclidean_distance (float current_x, float current_y, float future_x , float future_y){
    distance = sqrt( (current_x-future_x) * (current_x-future_x) + (current_y - future_y) * (current_y - future_y) );
    //cout << "Distanz: " << distance << endl; 
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


bool PixelIsFrontier (int currentpos, std::vector<signed char> data_current, nav_msgs::MapMetaData info_current){

  int free_pixel = 0;
  int occupied_pixel = 100;
  int unexplored_pixel = -1;

  if( (data_current[currentpos]==free_pixel)  && ( (data_current[currentpos-1] == unexplored_pixel) || 
      (data_current[currentpos+1] == unexplored_pixel) || (data_current[currentpos - info_current.width]==unexplored_pixel)|| 
      (data_current[currentpos + info_current.width]==unexplored_pixel)  ) )
  {

    return true;
  }

  else
  {
    return false;
  }

}


bool IsPixelUpLeftCorner (int corner_up_left, std::vector<signed char> data_up_left, nav_msgs::MapMetaData info_up_left){

  int free_pixel = 0;
  int occupied_pixel = 100;
  int unexplored_pixel = -1;

  if(corner_up_left == 0){

    if( (data_up_left[corner_up_left] ==free_pixel) && ( (data_up_left[corner_up_left+1] == unexplored_pixel) || (data_up_left[corner_up_left+info_up_left.width] == unexplored_pixel) ) ){

      return true;
    }
  }

  return false;

} 


bool IsPixelUpRightCorner (int corner_up_right, std::vector<signed char> data_up_right, nav_msgs::MapMetaData info_up_right){

  int free_pixel = 0;
  int occupied_pixel = 100;
  int unexplored_pixel = -1;

  if(corner_up_right == info_up_right.width-1){

    if( (data_up_right[corner_up_right] == free_pixel) && ((data_up_right[corner_up_right-1] == unexplored_pixel) || (data_up_right[corner_up_right+info_up_right.width] ==unexplored_pixel) ) ){

      return true;
    }
  }

  return false;

}


bool IsPixelDownLeftCorner (int corner_down_left, std::vector<signed char> data_down_left, nav_msgs::MapMetaData info_down_left){

  int free_pixel = 0;
  int occupied_pixel = 100;
  int unexplored_pixel = -1;

  if(corner_down_left == info_down_left.width * (info_down_left.height-1)){
  
    if( (data_down_left[corner_down_left] == free_pixel) && ( (data_down_left [corner_down_left-info_down_left.width] == unexplored_pixel) || (data_down_left[corner_down_left+1] == unexplored_pixel))){

      return true;
    }
  }

  return false;

}


bool IsPixelDownRightCorner (int corner_down_right, std::vector<signed char> data_down_right, nav_msgs::MapMetaData info_down_right){

  int free_pixel = 0;
  int occupied_pixel = 100;
  int unexplored_pixel = -1;

  if(corner_down_right == info_down_right.width * info_down_right.height-1 ){

    if( (data_down_right[corner_down_right] == free_pixel) && ((data_down_right[corner_down_right-1] == unexplored_pixel) || (data_down_right[corner_down_right-info_down_right.width] == unexplored_pixel) ) ){

      return true;
    }
  }

  return false;

}


bool IsPixelFirstLine (int first_line, std::vector<signed char> data_first_line, nav_msgs::MapMetaData info_first_line){

  int free_pixel = 0;
  int occupied_pixel = 100;
  int unexplored_pixel = -1;

  if( (first_line<info_first_line.width-1)&&(first_line!=0) ){

    if( (data_first_line[first_line] == free_pixel) && ((data_first_line[first_line-1] ==unexplored_pixel) || (data_first_line[first_line+1] ==unexplored_pixel) || (data_first_line[first_line+info_first_line.width] ==unexplored_pixel) ) ){

      return true;
    }
  }

  return false;

}


bool IsPixelLastLine (int last_line, std::vector<signed char> data_last_line, nav_msgs::MapMetaData info_last_line){

  int free_pixel = 0;
  int occupied_pixel = 100;
  int unexplored_pixel = -1;


  if( (last_line >info_last_line.width *(info_last_line.height-1)) && (last_line< info_last_line.width * info_last_line.height-1) ){

    if( (data_last_line[last_line] ==free_pixel) && ((data_last_line[last_line-1] ==unexplored_pixel) || (data_last_line[last_line-info_last_line.width] ==unexplored_pixel) || (data_last_line[last_line+1] ==unexplored_pixel) ) ){

      return true;
    }
  }

  return false;

}


bool IsPixelFirstColumn (int first_column, std::vector<signed char> data_first_column, nav_msgs::MapMetaData info_first_column){

  int free_pixel = 0;
  int occupied_pixel = 100;
  int unexplored_pixel = -1;

  if( (first_column % info_first_column.width == 0) && (first_column!=0) && (first_column != info_first_column.width * (info_first_column.height-1) ) ){

    if  ( (data_first_column[first_column] ==free_pixel) &&( (data_first_column[first_column-info_first_column.width] ==unexplored_pixel) || (data_first_column[first_column+1] == unexplored_pixel) || (data_first_column[first_column+info_first_column.width] == unexplored_pixel))){

      return true;
    }

  }

  return false;

}


bool IsPixelLastColumn (int last_column, std::vector<signed char> data_last_column, nav_msgs::MapMetaData info_last_column){

  int free_pixel = 0;
  int occupied_pixel = 100;
  int unexplored_pixel = -1;

  if( (last_column % info_last_column.width == info_last_column.width -1) && (last_column!=info_last_column.width-1) && (last_column != info_last_column.width * info_last_column.height -1)){

    if( (data_last_column[last_column] ==free_pixel) && ((data_last_column[last_column-info_last_column.width] ==unexplored_pixel) || (data_last_column[last_column-1] ==unexplored_pixel) ||  (data_last_column[last_column+1] ==unexplored_pixel) )){

      return true;
    }
  }

  return false;

}



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

  int distance_of_frontier = 0;
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
  int occupied_pixel = 100;
  int unexplored_pixel = -1;

  int right = counter + 1;
  int left = counter - 1;
  int up = counter - info_map.width;
  int down = counter + info_map.width;
  int leftup = counter - info_map.width-1;
  int rightup = counter  - info_map.width+1;
  int leftdown = counter + info_map.width-1;
  int rightdown = counter  + info_map.width+1;
  Frontiers collectedfrontiers = transfer;
  
  collectedfrontiers.set_pixels(counter);

  //anze++;
  //cout << anze <<". Rekursion" << endl;

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
    //std::cout << "Color: " << replacement << " at index: " << counter << std::endl; 
    FrontierMap.data[counter] = substitution;
   }
  
  if  ( (right % info_map.width != info_map.width -1)  ) 
  { 
    if (  PixelIsFrontier ( right, FrontierMap.data,  FrontierMap.info)){

      //cout << "rechts" << endl;
      collectedfrontiers=FloodfillFrontiers(right, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }
  
  
  if  ( left % info_map.width != 0 ) 
  { 
if (  PixelIsFrontier ( left, FrontierMap.data,  FrontierMap.info)){

      //cout << "links" << endl;
      collectedfrontiers=FloodfillFrontiers(left, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }
  

  if  ( up >= info_map.width -1 )   
  { 
  if (  PixelIsFrontier ( up, FrontierMap.data,  FrontierMap.info)){

      //cout << "oben" << endl;
      collectedfrontiers=FloodfillFrontiers(up, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }
  

  if   ( down <= info_map.width * info_map.height-1 )  
  { 
   if (  PixelIsFrontier ( down, FrontierMap.data,  FrontierMap.info)){

      //cout << "unten" << endl;
      collectedfrontiers=FloodfillFrontiers(down, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }
 

  if  ( leftup >= info_map.width -1 )  
  { 
 if (  PixelIsFrontier ( leftup, FrontierMap.data,  FrontierMap.info)){

      //cout << "linksoben" << endl;
      collectedfrontiers=FloodfillFrontiers(leftup, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }


  if  ( rightup >= info_map.width -1 )  
  { 
  if (  PixelIsFrontier ( rightup, FrontierMap.data,  FrontierMap.info)){

      //cout << "rechtsoben" << endl;
      collectedfrontiers=FloodfillFrontiers(rightup, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }


  if  ( leftdown <= info_map.width * info_map.height-1 )  
  { 
 if (  PixelIsFrontier ( leftdown, FrontierMap.data,  FrontierMap.info)){

      //cout << "linksunten" << endl;
      collectedfrontiers=FloodfillFrontiers(leftdown, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }
  

  if ( rightdown <= info_map.width * info_map.height-1 )  
  { 
   if (  PixelIsFrontier ( rightdown, FrontierMap.data,  FrontierMap.info)){

      //cout << "rechtsunten" << endl;
      collectedfrontiers=FloodfillFrontiers(rightdown, substitution, FrontierMap.data, FrontierMap.info, collectedfrontiers);
    }
  }


  //cout << anze <<". Rekursion beendet" << endl;
 
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
  // vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 1); 
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

    // ros::spinOnce(); 
    // vis_pub.publish( marker );
    // }

  //ros::spin();

  return 0;
  
}

//Function for the received map data with the parameter nav_msgs --> OccupancyGrid
void mapCallback(const nav_msgs:: OccupancyGrid::ConstPtr& msg)
{ 
  tf::TransformListener listener;

  ros::Rate rate(10.0);
  //while (nh.ok()){
    tf::StampedTransform transform;
      try{
        listener.waitForTransform("/map", "/robot0", ros::Time(), ros::Duration(4.0));
        listener.lookupTransform( "/map" , "/robot0",ros::Time(0), transform);
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        //continue;
    }
  //}
  //Saving the parts of the OccupancyGrid in variables --> see documentation: doc.ros.org
  std_msgs::Header header_old = msg->header;
  nav_msgs::MapMetaData info_old = msg->info;
  std::vector<signed char> data_old = msg->data;
  std::vector <Frontiers> every_frontier;
  
  
  FrontierMap.header = header_old;
  FrontierMap.info = info_old;
  FrontierMap.data = data_old; 

  bool pixel_checked = true;

  Frontiers addingfrontier;

  int replacement = 10;
  
  
  //Show the height, width and size of the map
  ROS_INFO("Got map %d, %d", info_old.width, info_old.height);
  ROS_INFO_STREAM("size of map " << data_old.size());

  // 1. routine für abfrage der vier nachbarn schreiben
  // 2. schleife über alle zellen
  // 3. publisher für neue karte
  // 4. überlegen wie grenzen gefunden werden können (zellebene)
  for (int i =0; i < info_old.width *info_old.height; i++) {

    // ROS_ERROR_STREAM("current color: " << replacement); 

    //Pixel at the up-left corner

    if( IsPixelUpLeftCorner(i, FrontierMap.data, FrontierMap.info) == true){
      //ROS_INFO("The neighbours of the up-left corner are  %d, %d \n", i+1 , i+info_old.width);
      //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
      pixel_checked = ExistedFrontier(i, every_frontier); 

      if (pixel_checked==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center();
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        every_frontier.push_back(addingfrontier);
        addingfrontier.delete_pixels();
        replacement = replacement + 10;
      }
      
      continue;
    }
    

    //Pixel at the up-right corner

    if(IsPixelUpRightCorner(i, FrontierMap.data, FrontierMap.info) == true){
      //ROS_INFO("The neighbours of the up-right corner are  %d, %d \n", i-1,  i+info_old.width);
      //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
      pixel_checked = ExistedFrontier(i, every_frontier);

      if (pixel_checked==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center();
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        every_frontier.push_back(addingfrontier);
        addingfrontier.delete_pixels();
        replacement = replacement + 10;
      }
      
      continue;
    }
       
    //Pixels in the first line--> not included the up left/ up-right corner

    if( IsPixelFirstLine (i, FrontierMap.data, FrontierMap.info) == true){
      // ROS_INFO("The neighbours in the first line (without up left/right corner) are  %d, %d, %d \n", i-1, i+1 , i+info_old.width);
      // ROS_INFO_STREAM("pixel" << i <<" is a boundary");
      pixel_checked = ExistedFrontier(i,every_frontier);

      if (pixel_checked==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center();
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        every_frontier.push_back(addingfrontier);
        replacement = replacement + 10;
        addingfrontier.delete_pixels();
      }
      
      continue;
    }

    // Pixels in the first column --> not included up-left / down left corner

    if( IsPixelFirstColumn (i, FrontierMap.data, FrontierMap.info) == true){
      //ROS_INFO("The neighbours  in the first column (wihtout up/down left corner) are  %d, %d, %d \n", i-info_old.width, i+1, i+info_old.width  );  
      //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
      pixel_checked = ExistedFrontier(i,every_frontier);

      if (pixel_checked==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center();
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        every_frontier.push_back(addingfrontier);
        replacement = replacement + 10;
        addingfrontier.delete_pixels();
      }
      
      continue;
      
    }
    
    // Pixel at the down-left corner

    if(IsPixelDownRightCorner (i, FrontierMap.data, FrontierMap.info) == true){
      // ROS_INFO("The neighbours of the down-left corner are  %d, %d \n", i-info_old.width, i+1  );
      // ROS_INFO_STREAM("pixel" << i <<" is a boundary");
      pixel_checked = ExistedFrontier(i,every_frontier);

      if (pixel_checked==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center();
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        every_frontier.push_back(addingfrontier);
        replacement = replacement + 10;
        addingfrontier.delete_pixels();
      }
    
      continue;
    }

    // Pixels in the last line--> not included the down-left / down right corner

    if( IsPixelLastLine (i, FrontierMap.data, FrontierMap.info) == true ){
      // ROS_INFO("The neighbours in the last line (without down-left/right corner) are  %d, %d, %d \n", i-1,  i-info_old.width, i+1 );
      // ROS_INFO_STREAM("pixel" << i <<" is a boundary");
      pixel_checked = ExistedFrontier(i,every_frontier);

      if (pixel_checked==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center();
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        every_frontier.push_back(addingfrontier);
        replacement = replacement + 10;
        addingfrontier.delete_pixels();
      }
      
      continue;
    }

    // Pixels in the last column --> not included the up-right / down right corner

    if ( IsPixelLastColumn (i, FrontierMap.data, FrontierMap.info) == true  ){
      // ROS_INFO("The neighbours in the last column (without up/down right corner) are  %d, %d, %d \n", i-info_old.width, i-1, i+info_old.width  );
      // ROS_INFO_STREAM("pixel" << i <<" is a boundary");
      pixel_checked = ExistedFrontier(i,every_frontier);

      if (pixel_checked==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center();
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        every_frontier.push_back(addingfrontier);
        replacement = replacement + 10;
        addingfrontier.delete_pixels();
      }

      continue;
    }

    //Pixel at the down-right corner

    if(IsPixelDownRightCorner (i, FrontierMap.data, FrontierMap.info) == true ){
      // ROS_INFO_STREAM("The neighbours are of the down-right corner " << i-1 << " " << i-info_old.width << "\n");
      // ROS_INFO_STREAM("pixel" << i <<" is a boundary");
      pixel_checked = ExistedFrontier(i,every_frontier);

      if (pixel_checked==false) {
        addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
        addingfrontier.pixels_x_y_transformation(FrontierMap.info);
        addingfrontier.calc_gravity_of_center();
        addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
        every_frontier.push_back(addingfrontier);
        replacement = replacement + 10;
        addingfrontier.delete_pixels();
      }

      continue;
    }

    //Other Pixels

    // ROS_INFO_STREAM("The neighbours are "<< i-1 <<" " << i-info_old.width <<" "<< i+1 <<" "<< i+info_old.width<<" "<<  "\n"  );
    if ( PixelIsFrontier ( i, FrontierMap.data,  FrontierMap.info)  ){
      //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
      pixel_checked = ExistedFrontier(i,every_frontier);

    if (pixel_checked==false) {
      addingfrontier = FloodfillFrontiers(i, replacement, FrontierMap.data, FrontierMap.info, addingfrontier);
      addingfrontier.pixels_x_y_transformation(FrontierMap.info);
      addingfrontier.calc_gravity_of_center();
      addingfrontier.euclidean_distance ( transform.getOrigin().x(), transform.getOrigin().y(), addingfrontier.get_gravity_of_center_x() , addingfrontier.get_gravity_of_center_x());
      every_frontier.push_back(addingfrontier);
      addingfrontier.delete_pixels();
      replacement = replacement + 10;
      }
    }

  }
 
  // for (int iter = 0; iter < every_frontier.size()  ; iter++){  
  //   every_frontier[iter].print_pixels();
  //   cout << "Ende des Frontiers" <<endl;  
  // }

  cout << every_frontier.size() << endl;

  std::cout << FrontierMap.data.size() << std::endl; 
  
  // for(size_t i=0 ; i< FrontierMap.info.width ; i++)
  // {
  //   FrontierMap.data[i] = 200; 
  // }

    // Initialization for the ROS-Publisher to publish the new map data
    map_pub.publish(FrontierMap);
    ROS_INFO_STREAM("Publishing the NewMap.\n" );
    sendingcoor(BiggestFrontier( every_frontier), every_frontier);
    every_frontier.clear();
}



void sendingcoor(int current ,std::vector <Frontiers> every_frontier){

 //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
   
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = every_frontier[current].get_gravity_of_center_x(); 
  goal.target_pose.pose.position.y = every_frontier[current].get_gravity_of_center_y();
  goal.target_pose.pose.orientation.w = 1.0;
  goal.target_pose.pose.orientation.z = 1.0;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
     ROS_INFO("The base failed to move forward 1 meter for some reason");

}


