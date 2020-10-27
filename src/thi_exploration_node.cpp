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
    int number_of_pixel;
    int number_of_diagonals;

  public:

    Frontiers(){
      cout << "object created" <<endl;
      number_of_pixel = 0;
      number_of_diagonals = 0;

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
    
    bool pixel_item_exists(int number){
      for (int j = 0; j < pixel_length.size(); j++)
      { 
        if(pixel_length[j]==number){
          //cout<< "Pixel is in Frontiers " << endl;
          return true;
        }
      }
      //cout<< "Pixel is not in Frontiers " << endl;
      return false;
    }

    int get_pixel_size(){
      number_of_pixel = pixel_length.size();
      return number_of_pixel;
    }

    int get_diagonals(){
      return number_of_diagonals;
    }

    void diagonal_registered(){
      number_of_diagonals = number_of_diagonals + 1;
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
int replacement =10;
nav_msgs:: OccupancyGrid FrontierMap;
// FrontierMap.header;   
// FrontierMap.info;
// FrontierMap.data;

//Function for the received map data
void mapCallback(const nav_msgs:: OccupancyGrid::ConstPtr& msg);



//Parameter std::vector with the mapdata, nav msgs:: mapinfo and the Position of the Pixel
void SeachringFrontiers( std::vector<signed char> data_map, nav_msgs::MapMetaData info_map , int counter)
{
  int right = counter + 1;
  int left = counter - 1;
  int up = counter - info_map.width;
  int down = counter + info_map.width;
  int leftup = counter - info_map.width-1;
  int rightup = counter  - info_map.width+1;
  int leftdown = counter + info_map.width-1;
  int rightdown = counter  + info_map.width+1;
  Frontiers collectedfrontiers;
  
  collectedfrontiers.set_pixel_length(counter);
  

  cout << "Start to look for frontiers:\n" << endl;
  
  //Loop for the Frontier pixels in the right-line  
   while (( right % info_map.width != info_map.width -1)  && (data_map[right]!=occupied_pixel)){
    //cout << "rechts" << endl;
    collectedfrontiers.set_pixel_length(right);
    right++;
    }


  //Loop for the Frontier pixels in the left-line
  while (( left % info_map.width != 0 ) && (data_map[left]!=occupied_pixel)){
    collectedfrontiers.set_pixel_length(left);
    //cout << "links" << endl;
    left--;
  } 

  
  //Loop for the Frontier pixels in the up-column
  while ( ( up >= info_map.width -1 ) && (data_map[up]!=occupied_pixel)){
    collectedfrontiers.set_pixel_length(up);
    //cout << "oben" << endl;
    up = up - info_map.width;
  }

  //Loop for the Frontier pixels in the down-column
  while (( down <= info_map.width * info_map.height-1 ) && (data_map[down]!=occupied_pixel)){
    collectedfrontiers.set_pixel_length(down);
    //cout << "unten" << endl;
    down = down + info_map.width;
  } 
  
  //Loop for the Frontier pixels in the left-up-diagonal-line
  while (( leftup >= info_map.width -1 ) && (data_map[leftup]!=occupied_pixel)){
    collectedfrontiers.set_pixel_length(leftup);
    leftup = leftup - info_map.width-1;
    //cout << "linksoben" << endl;
    collectedfrontiers.diagonal_registered();
  }
  
  //Loop for the Frontier pixels in the right-up-diagonal-line
  while (( rightup >= info_map.width -1 ) && (data_map[rightup]!=occupied_pixel)){
    collectedfrontiers.set_pixel_length(rightup);
    rightup = rightup - info_map.width+1;
    //cout << "rechtsoben" << endl;
    collectedfrontiers.diagonal_registered();
  } 

  //Loop for the Frontier pixels in the left-down-diagonal-line
  while (( leftdown <= info_map.width * info_map.height-1 ) && (data_map[leftdown]!=occupied_pixel)) {
    collectedfrontiers.set_pixel_length(leftdown);
    leftdown = leftdown + info_map.width-1;
    //cout << "linksunten" << endl;
    collectedfrontiers.diagonal_registered();
  }
 
  //Loop for the Frontier pixels in the Right-down-diagonal-line
  while (( rightdown <= info_map.width * info_map.height-1 ) && (data_map[rightdown]!=occupied_pixel)){
    collectedfrontiers.set_pixel_length(rightdown);
    rightdown = rightdown + info_map.width+1;
    //cout << "rechtsunten" << endl;
    collectedfrontiers.diagonal_registered();
  } 
  anze++;
  every_frontier.push_back(collectedfrontiers);
  cout << "All Frontiers of the Pixel "<< counter << " were found: \n" << endl;
  cout<< "Anzahl der Aufrufe " << anze << endl;
  //collectedfrontiers.print_pixel_length();     
        
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
  
  //std::vector<signed char> data_frontier =data_map;
  int right = counter + 1;
  int left = counter - 1;
  int up = counter - info_map.width;
  int down = counter + info_map.width;
  int leftup = counter - info_map.width-1;
  int rightup = counter  - info_map.width+1;
  int leftdown = counter + info_map.width-1;
  int rightdown = counter  + info_map.width+1;
  Frontiers collectedfrontiers = transfer;
  
  FrontierMap.info = info_map;
  FrontierMap.data = data_map;

  collectedfrontiers.set_pixel_length(counter);

  anze++;
  cout << anze <<". Rekursion" << endl;

   if (FrontierMap.data[counter] == replacement) {
    cout << "Wenn die gewünschte Farbe im Pixel schon ersetzt wurde" << endl;
    return collectedfrontiers;
  }

  else if  ( (FrontierMap.data[counter] != free_pixel ) && (FrontierMap.data[counter] != unexplored_pixel ) ){
    cout << "Wenn gesuchte Farbe nicht im Pixel steht" << endl;
    return collectedfrontiers;
  }
  
  else { 
    cout << "Ersetze im Pixel die gewünschte Farbe" << endl;
    //data_map[counter]=replacement;
    FrontierMap.data[counter] = replacement;
   }
  
  if  ( (right % info_map.width != info_map.width -1)  &&  (FrontierMap.data[right]==free_pixel)  && ( (FrontierMap.data[right+1] == unexplored_pixel)
      || (FrontierMap.data[right - info_map.width]==unexplored_pixel) || (FrontierMap.data[right + info_map.width]==unexplored_pixel)  ) )
  { 
    cout << "rechts" << endl;
    collectedfrontiers=FloodfillFrontiers(right, FrontierMap.data, info_map, collectedfrontiers);
  }
  
  if ( ( left % info_map.width != 0 )  &&  (FrontierMap.data[left]==free_pixel) && ( (FrontierMap.data[left-1] == unexplored_pixel )
      || (FrontierMap.data[left - info_map.width]==unexplored_pixel) || (FrontierMap.data[left + info_map.width]==unexplored_pixel) ) )
  { 
    cout << "links" << endl;
    collectedfrontiers=FloodfillFrontiers(left, FrontierMap.data, info_map, collectedfrontiers);
  }
  
  if ( ( up <= info_map.width -1 )  &&  (FrontierMap.data[up]==free_pixel) && ( (FrontierMap.data[up-info_map.width] == unexplored_pixel )
      || (FrontierMap.data[up +1]==unexplored_pixel) || (FrontierMap.data[up - 1]==unexplored_pixel) ) )
  { 
    cout << "oben" << endl;
    collectedfrontiers = FloodfillFrontiers(up, FrontierMap.data, info_map, collectedfrontiers);
  }
  
  if  ( ( down >= info_map.width * info_map.height-1 ) &&  (FrontierMap.data[down]==free_pixel)  && ( (FrontierMap.data[down+info_map.width] == unexplored_pixel )
      || (FrontierMap.data[down +1]==unexplored_pixel) || (FrontierMap.data[down - 1]==unexplored_pixel) ) ) 
  { 
    cout << "unten" << endl;
    collectedfrontiers = FloodfillFrontiers(down, FrontierMap.data, info_map, collectedfrontiers);
  }
 
  if ( ( leftup <= info_map.width -1 ) &&  (FrontierMap.data[leftup]==free_pixel)  && ( (FrontierMap.data[leftup+info_map.width] == unexplored_pixel ) ||
      (FrontierMap.data[leftup-info_map.width] == unexplored_pixel ) || (FrontierMap.data[leftup +1]==unexplored_pixel) || (FrontierMap.data[leftup - 1]==unexplored_pixel) ) ) 
  { 
    cout << "linksoben" << endl;
    collectedfrontiers = FloodfillFrontiers(leftup, FrontierMap.data, info_map, collectedfrontiers);
  }

  if ( ( rightup <= info_map.width -1 ) &&  (FrontierMap.data[rightup]==free_pixel) && ( (FrontierMap.data[rightup+info_map.width] == unexplored_pixel ) ||
      (FrontierMap.data[rightup-info_map.width] == unexplored_pixel ) || (FrontierMap.data[rightup +1]==unexplored_pixel) || (FrontierMap.data[rightup - 1]==unexplored_pixel) ) ) 
  { 
    cout << "rechtsoben" << endl;
    collectedfrontiers = FloodfillFrontiers(rightup, FrontierMap.data, info_map, collectedfrontiers );
  }
  
   if ( ( leftdown >= info_map.width * info_map.height-1 ) &&  (FrontierMap.data[leftdown]==free_pixel) && ( (FrontierMap.data[leftdown+info_map.width] == unexplored_pixel ) ||
      (FrontierMap.data[leftdown-info_map.width] == unexplored_pixel ) || (FrontierMap.data[leftdown +1]==unexplored_pixel) || (FrontierMap.data[leftdown - 1]==unexplored_pixel) ) ) 
  { 
    cout << "linksunten" << endl;
    collectedfrontiers = FloodfillFrontiers(leftdown, FrontierMap.data, info_map,collectedfrontiers );
  }
  
  if ( ( rightdown >= info_map.width * info_map.height-1 ) &&  (FrontierMap.data[rightdown]==free_pixel) && ( (FrontierMap.data[rightdown+info_map.width] == unexplored_pixel ) ||
      (FrontierMap.data[rightdown-info_map.width] == unexplored_pixel ) || (FrontierMap.data[rightdown +1]==unexplored_pixel) || (FrontierMap.data[rightdown - 1]==unexplored_pixel) ) ) 
  { 
    cout << "rechtsunten" << endl;
    collectedfrontiers= FloodfillFrontiers(rightdown, FrontierMap.data, info_map, collectedfrontiers );
  }

  cout << anze <<". Rekursion beendet" << endl;
 
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

  FrontierMap.header = header_old; 

  // nav_msgs:: OccupancyGrid NewMap;
  // NewMap.header = header_old;  
  // NewMap.info = info_old;
  // NewMap.data = msg->data;

  bool pixel_checked = true;

  Frontiers addingfrontier;

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
        pixel_checked = ExistedFrontier(i); 

        if (pixel_checked==false) {
          // SeachringFrontiers( data, info_old , i);
          // NewMap.data[i] =100;
          addingfrontier = FloodfillFrontiers(i, data, info_old, addingfrontier);
          every_frontier.push_back(addingfrontier);
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
          // SeachringFrontiers( data, info_old , i);
          // NewMap.data[i] = 100;
          addingfrontier = FloodfillFrontiers(i, data, info_old, addingfrontier);
          every_frontier.push_back(addingfrontier);
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
          // SeachringFrontiers( data, info_old , i);
          // NewMap.data[i] = 100;
          addingfrontier = FloodfillFrontiers(i, data, info_old, addingfrontier);
          every_frontier.push_back(addingfrontier);
          replacement = replacement + 10;
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
          // SeachringFrontiers( data, info_old , i);
          // NewMap.data[i] = 100;
          addingfrontier = FloodfillFrontiers(i, data, info_old, addingfrontier);
          every_frontier.push_back(addingfrontier);
          replacement = replacement + 10;
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
          // SeachringFrontiers( data, info_old , i);
          // NewMap.data[i] = 100;
          addingfrontier = FloodfillFrontiers(i, data, info_old, addingfrontier);
          every_frontier.push_back(addingfrontier);
          replacement = replacement + 10;
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
          // SeachringFrontiers( data, info_old , i);
          // NewMap.data[i] = 100;
          addingfrontier = FloodfillFrontiers(i, data, info_old, addingfrontier);
          every_frontier.push_back(addingfrontier);
          replacement = replacement + 10;
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
          // SeachringFrontiers( data, info_old , i);
          // NewMap.data[i] = 100;
          addingfrontier = FloodfillFrontiers(i, data, info_old, addingfrontier);
          every_frontier.push_back(addingfrontier);
          replacement = replacement + 10;
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
          // SeachringFrontiers( data, info_old , i);
          // NewMap.data[i] = 100;
          addingfrontier = FloodfillFrontiers(i, data, info_old, addingfrontier);
          every_frontier.push_back(addingfrontier);
          replacement = replacement + 10;
        }
      }

      else{
        // NewMap.data[i] =0;
      }
      
      continue;
    }

    //Other Pixels

    // ROS_INFO_STREAM("The neighbours are "<< i-1 <<" " << i-info_old.width <<" "<< i+1 <<" "<< i+info_old.width<<" "<<  "\n"  );
    if( (msg->data[i] ==free_pixel) &&( (msg->data[i-1] == unexplored_pixel) || (msg->data[i-info_old.width] == unexplored_pixel) ||  (msg->data[i+1] ==unexplored_pixel) || (msg->data[i+info_old.width] ==unexplored_pixel) )){
      //ROS_INFO_STREAM("pixel" << i <<" is a boundary");
      pixel_checked = ExistedFrontier(i);

      if (pixel_checked==false) {
        // SeachringFrontiers( data, info_old , i);
        // NewMap.data[i] = 100;
        addingfrontier = FloodfillFrontiers(i, data, info_old, addingfrontier);
        every_frontier.push_back(addingfrontier);
        replacement = replacement + 10;
      }
    }

    else{
      // NewMap.data[i] =0;
    }
  }

  // Initialization for the ROS-Publisher to publish the new map data
  ROS_INFO_STREAM("Publishing the NewMap.\n"  );


     
  // for (int iter = 0; iter < every_frontier.size(); iter++){
  //   cout<< "Every Frontiers:"<< endl;
  //   every_frontier[iter].print_pixel_length();   
  // }
  cout << every_frontier.size() << endl;

  map_pub.publish(FrontierMap);
}



