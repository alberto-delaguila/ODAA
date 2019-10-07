/*
 * Node: laser_clustering
 * Alberto del Águila Gómez, July 2019
 * 
 * 
 * This node is used for environment perception using a 2D laser. It takes a single reading, and segments data
 * into objects, which are then published to the corresponding topic. 
 */



//----LIBRARY INCLUDES---------------
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <stdlib.h>
#include <math.h>

//----ROS CUSTOM MESSAGES INCLUDES---
#include "detect_avoid/ObstacleList.h"

using namespace std;
//----CONSTANTS (TO BE TRANSFORMED INTO ROS PARAMS)
/*
 * DIFFERENCE_TRESHOLD: difference limit between to points for being considered part of the same object
 * LASER_RANGE: distance limit for a point to be processed. All points above this limit are counted as out of range, altough it may not be
 * 	physically out of range.
 * FREC: maximum rate for main loop execution
 */
const float DIFFERENCE_TRESHOLD = 1.0;
const float LASER_RANGE = 8;
const int FREC = 10;

//----DATA STRUCTURES AND TYPEDEFS----------------
/*
 * obstacle_t is an struct containing obstacle data:
 *   id: is used to index the object, although it may vary between scans
 *   n: stores the array position of the closest point of that object. It may be transformed later into angle information
 *   d: stores the distance of the closes point of that object.
 */
struct obstacle_t{
  uint id;
  uint n;
  float d;
};

//----GLOBAL VARIABLES---------------
ros::Publisher pub;
uint scan_id = 1;

//----FUNCTION PROTOTYPES------------
void cb_scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);


int main(int argc, char **argv)
{
  /*
   * ROS INITIALIZATION
   * With ros::init, this node starts the comunication with master.
   * ros::NodeHandle allows publication and subscription, among other features.
   * The advertise() method allows publication into the "obstacle_data" topic using a ros::Publisher object declared above.
   * ros::Rate limits the execution speed of this node
   * ros::Subscriber creates an object that allows subscription to the "scan" topic
   */
  ros::init(argc,argv,"laser_clust"); 
  ros::NodeHandle n;
  pub = n.advertise<detect_avoid::ObstacleList>("obstacle_data",1000);
  ros::Rate loop_rate(FREC); 
  ros::Subscriber sub = n.subscribe("scan",1000, cb_scanCallback);

  ROS_INFO("[INFO]laser_read -> NODE 'LASER_CLUSTERING' STARTED");
  
  /*
   * This while loop contains the main execution part of the node, which runs while the master is live and ros::shutdown is not
   * called. With ros::spinOnce, all topics are checked for new messages, and if so, the attached callback function is called. This method
   * only does this action once, while ros::spin() does the same process indefinitely, blocking the program flow. 
   * Also, the rate is controlled by the sleep() method, which returns true if that rate is achieved. This is used to inform the user
   * that the code is too slow or something went wrong.
   */
  while(ros::ok())
  {
    ros::spinOnce();
    if(!loop_rate.sleep()) ROS_INFO("[WARN]laser_read -> TIME CONDITION OF %d HZ NOT MET",FREC);

  }
  return 0;
}

void cb_scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan)
{
  ROS_INFO("SCAN ID %d",scan_id);
  detect_avoid::ObstacleList obst_data;
  vector<obstacle_t> obst;
  obstacle_t temp;
  scan_id++;
  
  temp.id = 1;
  
  /*
   * This callback function contains the algoritm that segments the laser data, and transforms it into object data
   */
  
  if(scan->ranges[0] < LASER_RANGE)
  {
    /*
     * Here, the first position of the array is checked, as the main algoritm starts at the second position
     */
    temp.d = scan->ranges[0];
    temp.n = 0;
    //ROS_INFO("[DEBUG]laser_read/cb_scanCallback -> OBJECT %d CREATED: [%.3f,%d]",temp.id,temp.d,temp.n);
  }
  
  for (uint i=1; i < scan->ranges.size();i++)
  {
    /*
     * For the rest of positions, some conditions are checked:
     *  - The distance is less than a fixed range. This filters out readings that return "inf" (out of laser physical range)
     *  - If the difference between the current distance and the last one is less than a fixed treshold, the point belongs to the same object,
     * 	  and that object's data is updated if needed.
     *  - If not, the point belongs to a new object, so the former is saved into an array, and current object data is reset.
     */
    if(scan->ranges[i] < LASER_RANGE && scan->ranges[i] > 0)
    {
      if (abs(scan->ranges[i] - scan->ranges[i-1]) < DIFFERENCE_TRESHOLD && scan->ranges[i-1] < LASER_RANGE)
      {
	if(scan->ranges[i] < temp.d)
	{ 
	  temp.d = scan->ranges[i];
	  temp.n = i;
	  //ROS_INFO("[DEBUG]laser_read/cb_scanCallback -> OBJECT %d UPDATED: [%.3f,%d]",temp.id,temp.d,temp.n);
	}
      }
      else
      {
	obst.push_back(temp);
	
	temp.id++;
	temp.d = scan->ranges[i];
	temp.n = i;
	//ROS_INFO("[DEBUG]laser_read/cb_scanCallback -> OBJECT %d CREATED: [%.3f,%d]",temp.id,temp.d,temp.n);
      }
     }
    }
  /*
   * The last object is saved, and the number of objects is set.
   */
  obst.push_back(temp);
  obst_data.count = obst.size();
  
  /*
   * Object data is transformed into the obstace data needed by the avoidance node.
   * This includes the scan id (for error tracking), the angle in radians, and the minimum distance to the object
   */
  for(int i=0; i< obst.size(); i++)
  {  
      obst_data.id = scan_id;
      obst_data.a.push_back(obst[i].n * scan->angle_increment);
      obst_data.d.push_back(obst[i].d);
      ROS_INFO("[INFO]laser_read/cb_scanCallback -> OBSTACLE %d AT %.3f, D=%.3f", obst[i].id,obst[i].n * scan->angle_increment,obst[i].d);
  }
  pub.publish(obst_data);
}