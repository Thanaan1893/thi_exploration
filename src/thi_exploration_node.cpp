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
    std:: vector< int> pixels;
    int number_of_pixel;
    int number_of_diagonals;

  public:

    Frontiers(){
      //cout << "object created" <<endl;
      number_of_pixel = 0;
      number_of_diagonals = 0;

    }

    void delete_pixels(){
      pixels.clear();
    }
    
    void set_pixels( int pos){
      pixels.push_back(pos);

    }

    void print_pixels(){
      for (int i = 0; i < pixels.size(); i++)
      {
        cout<< "Pixel in the Frontiers are: \n"<< pixels[i] <<endl;
      }
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

};

// global variables
//!< global variable to publish map
ros::Publisher map_pub;
int free_pixel = 0;
int occupied_pixel = 100;
int unexplored_pixel = -1;
std::vector <Frontiers> every_frontier;
int anze=0;
int replacement = 10;
nav_msgs:: OccupancyGrid FrontierMap;


//Function for the received map data with the parameter nav_msgs --> OccupancyGrid
void mapCallback(const nav_msgs:: OccupancyGrid::ConstPtr& msg);

bool PixelIsFrontier (int currentpos, std::vector<signed char> data_current, nav_msgs::MapMetaData info_current){

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


bool ExistedFrontier (int position){
     
  for (int iter = 0; iter < every_frontier.size(); iter++){
    if( every_frontier[iter].pixel_item_exists(position)== true){
      //cout<< "Pixel is in Every Frontiers " << endl;
      return true;
    } 
     
  }
  //cout<< "Pixel is not in Every Frontiers " << endl;
  return false;
}

int BiggestFrontier (){

  int size_of_frontier = 0;
  int frontier_pos =0;
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

int MostDiagonalsFrontier (){

  int diagonals_of_frontier = 0;
  int pos_frontier = 0;
  for (int iter = 0; iter < every_frontier.size(); iter++)
  {
    if( every_frontier[iter].get_diagonals() < diagonals_of_frontier)
    {
      diagonals_of_frontier=every_frontier[iter].get_diagonals();
      pos_frontier = iter;
    } 
     
  }
    return pos_frontier;
}

Frontiers FloodfillFrontiers(int counter, std::vector<signed char> data_map, nav_msgs::MapMetaData info_map, Frontiers transfer ){
  
  
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

  anze++;
  //cout << anze <<". Rekursion" << endl;

   if (FrontierMap.data[counter] == replacement) {
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
    FrontierMap.data[counter] = replacement;
   }
  
  if  ( (right % info_map.width != info_map.width -1) &&  (PixelIsFrontier ( right, FrontierMap.data,  FrontierMap.info) ) )
  { 
    //cout << "rechts" << endl;
    collectedfrontiers=FloodfillFrontiers(right, FrontierMap.data, FrontierMap.info, collectedfrontiers);
  }
  
  if ( ( left % info_map.width != 0 )  &&  (PixelIsFrontier ( left, FrontierMap.data,  FrontierMap.info) ) )
  { 
    //cout << "links" << endl;
    collectedfrontiers=FloodfillFrontiers(left, FrontierMap.data, FrontierMap.info, collectedfrontiers);
  }
  
  if ( ( up >= info_map.width -1 )  &&  (PixelIsFrontier ( up, FrontierMap.data,  FrontierMap.info) ) ) 
  { 
    //cout << "oben" << endl;
    collectedfrontiers = FloodfillFrontiers(up, FrontierMap.data, FrontierMap.info, collectedfrontiers);
  }
  
  if  ( ( down <= info_map.width * info_map.height-1 ) &&  (PixelIsFrontier ( down, FrontierMap.data,  FrontierMap.info) ) ) 
  { 
    //cout << "unten" << endl;
    collectedfrontiers = FloodfillFrontiers(down, FrontierMap.data, FrontierMap.info, collectedfrontiers);
  }
 
  if ( ( leftup >= info_map.width -1 ) &&  (PixelIsFrontier ( leftup, FrontierMap.data,  FrontierMap.info) ) ) 
  { 
    //cout << "linksoben" << endl;
    collectedfrontiers = FloodfillFrontiers(leftup, FrontierMap.data, FrontierMap.info, collectedfrontiers);
  }

  if ( ( rightup >= info_map.width -1 ) &&  (PixelIsFrontier ( rightup, FrontierMap.data,  FrontierMap.info) ) ) 
  { 
    //cout << "rechtsoben" << endl;
    collectedfrontiers = FloodfillFrontiers(rightup, FrontierMap.data, FrontierMap.info, collectedfrontiers );
  }
  
   if ( ( leftdown <= info_map.width * info_map.height-1 ) &&  (PixelIsFrontier ( leftdown, FrontierMap.data,  FrontierMap.info) ) ) 
  { 
    //cout << "linksunten" << endl;
    collectedfrontiers = FloodfillFrontiers(leftdown, FrontierMap.data, FrontierMap.info, collectedfrontiers );
  }
  
  if ( ( rightdown <= info_map.width * info_map.height-1 ) &&  (PixelIsFrontier ( rightdown, FrontierMap.data,  FrontierMap.info) ) ) 
  { 
    //cout << "rechtsunten" << endl;
    collectedfrontiers= FloodfillFrontiers(rightdown, FrontierMap.data, FrontierMap.info, collectedfrontiers );
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
  

  ros::spin(); 
  
  return 0;
  
}

//Function for the received map data with the parameter nav_msgs --> OccupancyGrid
void mapCallback(const nav_msgs:: OccupancyGrid::ConstPtr& msg)
{ 
  //Saving the parts of the OccupancyGrid in variables --> see documentation: doc.ros.org
  std_msgs::Header header_old = msg->header;
  nav_msgs::MapMetaData info_old = msg->info;
  std::vector<signed char> data_old = msg->data;

  
  FrontierMap.header = header_old;
  FrontierMap.info = info_old;
  FrontierMap.data = data_old; 

  // nav_msgs:: OccupancyGrid NewMap;
  // NewMap.header = header_old;  
  // NewMap.info = info_old;
  // NewMap.data = msg->data;

  bool pixel_checked = true;

  Frontiers addingfrontier;

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

    if(i==0){
      //ROS_INFO("The neighbours of the up-left corner are  %d, %d \n", i+1 , i+info_old.width);

      if( (msg->data[i] ==free_pixel) && ((msg->data[i+1] ==unexplored_pixel) || (msg->data[i+info_old.width] ==unexplored_pixel) )){
        //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
        pixel_checked = ExistedFrontier(i); 

        if (pixel_checked==false) {
          // NewMap.data[i] =100;
          addingfrontier = FloodfillFrontiers(i, FrontierMap.data, FrontierMap.info, addingfrontier);
          every_frontier.push_back(addingfrontier);
          addingfrontier.delete_pixels();
          replacement = replacement + 10;
        }
      }

      else{
        // NewMap.data[i] =0;
      }

      continue;
    }

    //Pixel at the up-right corner

    if(i==info_old.width-1){
      //ROS_INFO("The neighbours of the up-right corner are  %d, %d \n", i-1,  i+info_old.width);

      if( (msg->data[i] ==free_pixel) && ((msg->data[i-1] ==unexplored_pixel) || (msg->data[i+info_old.width] ==unexplored_pixel))){
        //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
        pixel_checked = ExistedFrontier(i);

        if (pixel_checked==false) {
          // NewMap.data[i] = 100;
          addingfrontier = FloodfillFrontiers(i, FrontierMap.data, FrontierMap.info, addingfrontier);
          every_frontier.push_back(addingfrontier);
          addingfrontier.delete_pixels();
          replacement = replacement + 10;
        }
      }

      else {
        // NewMap.data[i] =0;
      }
      
      continue;
    }
       
    //Pixels in the first line--> not included the up left/ up-right corner

    if( (i<info_old.width-1)&&(i!=0) ){
      //ROS_INFO("The neighbours in the first line (without up left/right corner) are  %d, %d, %d \n", i-1, i+1 , i+info_old.width);
        
      if( (msg->data[i] ==free_pixel) && ((msg->data[i-1] ==unexplored_pixel) || (msg->data[i+1] ==unexplored_pixel) || (msg->data[i+info_old.width] ==unexplored_pixel))){
        //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
        pixel_checked = ExistedFrontier(i);

        if (pixel_checked==false) {
          // NewMap.data[i] = 100;
          addingfrontier = FloodfillFrontiers(i, FrontierMap.data, FrontierMap.info, addingfrontier);
          every_frontier.push_back(addingfrontier);
          replacement = replacement + 10;
          addingfrontier.delete_pixels();
        }
      }

      else{
        // NewMap.data[i] =0;
      }
      continue;
      
    }

    // Pixels in the first column --> not included up-left / down left corner

    if( (i % info_old.width == 0) && (i!=0) && (i != info_old.width * (info_old.height-1) ) ){
      //ROS_INFO("The neighbours  in the first column (wihtout up/down left corner) are  %d, %d, %d \n", i-info_old.width, i+1, i+info_old.width  );
          
      if( (msg->data[i] ==free_pixel) &&( (msg->data[i-info_old.width] ==unexplored_pixel) || (msg->data[i+1] ==unexplored_pixel) || (msg->data[i+info_old.width] ==unexplored_pixel))){
        //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
        pixel_checked = ExistedFrontier(i);

        if (pixel_checked==false) {
          // NewMap.data[i] = 100;
          addingfrontier = FloodfillFrontiers(i, FrontierMap.data, FrontierMap.info, addingfrontier);
          every_frontier.push_back(addingfrontier);
          replacement = replacement + 10;
          addingfrontier.delete_pixels();
        }
      }

      else{
        // NewMap.data[i] =0;
      }
          
      continue;
      
    }
    
    // Pixel at the down-left corner

    if(i == info_old.width * (info_old.height-1)){
      //ROS_INFO("The neighbours of the down-left corner are  %d, %d \n", i-info_old.width, i+1  );

      if( (msg->data[i] ==free_pixel) && ((msg->data[i-info_old.width] ==unexplored_pixel) || (msg->data[i+1] ==unexplored_pixel)) ){
        //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
        pixel_checked = ExistedFrontier(i);

        if (pixel_checked==false) {
          // NewMap.data[i] = 100;
          addingfrontier = FloodfillFrontiers(i, FrontierMap.data, FrontierMap.info, addingfrontier);
          every_frontier.push_back(addingfrontier);
          replacement = replacement + 10;
          addingfrontier.delete_pixels();
        }
      }

      else{
        // NewMap.data[i] =0;
      }
      
      continue;
    }

    // Pixels in the last line--> not included the down-left / down right corner

    if( (i>info_old.width *(info_old.height-1)) && (i< info_old.width *info_old.height-1) ){
      //ROS_INFO("The neighbours in the last line (without down-left/right corner) are  %d, %d, %d \n", i-1,  i-info_old.width, i+1 );
        
      if( (msg->data[i] ==free_pixel) && ((msg->data[i-1] ==unexplored_pixel) || (msg->data[i-info_old.width] ==unexplored_pixel) || (msg->data[i+1] ==unexplored_pixel) ) ){
        //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
        pixel_checked = ExistedFrontier(i);

        if (pixel_checked==false) {
          // NewMap.data[i] = 100;
          addingfrontier = FloodfillFrontiers(i, FrontierMap.data, FrontierMap.info, addingfrontier);
          every_frontier.push_back(addingfrontier);
          replacement = replacement + 10;
          addingfrontier.delete_pixels();
        }
      }

      else{
        // NewMap.data[i] =0;
      }
       
      continue;
    }

    // Pixels in the last column --> not included the up-right / down right corner

    if ( (i % info_old.width == info_old.width -1) && (i!=info_old.width-1) && (i != info_old.width * info_old.height -1) ){
      //ROS_INFO("The neighbours in the last column (without up/down right corner) are  %d, %d, %d \n", i-info_old.width, i-1, i+info_old.width  );
       
      if( (msg->data[i] ==free_pixel) && ((msg->data[i-info_old.width] ==unexplored_pixel) || (msg->data[i-1] ==unexplored_pixel) ||  (msg->data[i+1] ==unexplored_pixel) )){
        //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
        pixel_checked = ExistedFrontier(i);

        if (pixel_checked==false) {
          // NewMap.data[i] = 100;
          addingfrontier = FloodfillFrontiers(i, FrontierMap.data, FrontierMap.info, addingfrontier);
          every_frontier.push_back(addingfrontier);
          replacement = replacement + 10;
          addingfrontier.delete_pixels();
          
        }
      }

      else{
        //  NewMap.data[i] =0;
      } 
      continue;
    }

    //Pixel at the down-right corner

    if(i == info_old.width * info_old.height -1){
      //ROS_INFO_STREAM("The neighbours are of the down-right corner " << i-1 << " " << i-info_old.width << "\n");
      
      if( (msg->data[i] ==free_pixel) && ((msg->data[i-1] ==unexplored_pixel) || (msg->data[i-info_old.width] ==unexplored_pixel) )){
        //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
        pixel_checked = ExistedFrontier(i);

        if (pixel_checked==false) {
          // NewMap.data[i] = 100;
          addingfrontier = FloodfillFrontiers(i, FrontierMap.data, FrontierMap.info, addingfrontier);
          every_frontier.push_back(addingfrontier);
          replacement = replacement + 10;
          addingfrontier.delete_pixels();
        }
      }

      else{
        // NewMap.data[i] =0;
      }
      
      continue;
    }

    //Other Pixels

    // ROS_INFO_STREAM("The neighbours are "<< i-1 <<" " << i-info_old.width <<" "<< i+1 <<" "<< i+info_old.width<<" "<<  "\n"  );
    if ( PixelIsFrontier ( i, FrontierMap.data,  FrontierMap.info)  ){
      //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
      pixel_checked = ExistedFrontier(i);

      if (pixel_checked==false) {
        // NewMap.data[i] = 100;
        addingfrontier = FloodfillFrontiers(i, FrontierMap.data, FrontierMap.info, addingfrontier);
        every_frontier.push_back(addingfrontier);
        addingfrontier.delete_pixels();
        replacement = replacement + 10;
      }
    }

    else{
      // NewMap.data[i] =0;
    }
  }

  // Initialization for the ROS-Publisher to publish the new map data
  ROS_INFO_STREAM("Publishing the NewMap.\n"  );
  
  //  int test = 50;
  //  for (int iter = 0; iter < every_frontier.size(); iter++){
  //     for (int k = 0; k < every_frontier[iter].get_pixel_size(); k++){
  //       FrontierMap.data[every_frontier[iter].get_pixel(k) ] = test;
  //    } 
  //     test= test + 10;
  //     every_frontier[iter].print_pixels();
  //     cout << "Ende des Frontiers" <<endl;
  //  }

  cout << every_frontier.size() << endl;

  std::cout << FrontierMap.data.size() << std::endl; 

  // for(size_t i=0 ; i< FrontierMap.info.width ; i++)
  // {
  //   FrontierMap.data[i] = 200; 
  // }


  map_pub.publish(FrontierMap);
}



