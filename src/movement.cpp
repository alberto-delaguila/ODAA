/*
 * Node: avoid_fms
 * Alberto del Águila Gómez, July 2019
 * 
 * 
 * This node is used for avoidance task. It takes data from the "obstacle_data" topic, checks the position of the obstacle
 * and generates an orientation reference for the car. Then, a closed loop controller makes the car to follow that reference.
 */


//----LIBRARY INCLUDES---------------
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Byte.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

//----ROS CUSTOM MESSAGES INCLUDES---
#include "detect_avoid/ObstacleList.h"


using namespace std;

//----CONSTANTS (TO BE TRANSFORMED INTO ROS PARAMS)
/* 
 * LATERAL_BOUNDARY: lateral maximum and minimun limit for an obstacle to be considered in the same lane, in a different lane, or out of the road
 * SECURE_DISTANCE: maximum frontal distance for an object to be considered an obstacle and be avoided.
 * GOAL_RECOMPUTE_DISANCE: distance at which the orientation reference should be recomputed after a lane change.
 * LATERAL_LOOKAHEAD: frontal distance for an object located at a lateral lane to be considered an obstacle.
 * ORIENTATION_ERROR: orientation error limit at which obstacles are being readed. Is used for avoiding errors in position calculation while
 * 		      the vehicle is changing lane or reorienting into its lane.
 * BACK_DISTANCE: distance from the rear of the vehicle to consider some object an obstacle.
 * EXTRA_LOOKAHEAD: additional lateral lookahead used in only the left side of the vehicle.
 * 
 * CONTROL_GAIN_x: controller gains.
 * NORMAL_SPEED: speed at which the car should navigate.
 * CHANGE_LANE_Q: angle reference increment or decrement for lane changing.
 * FREC: maximum rate for main loop execution.
 */
double LATERAL_BOUNDARY_MIN = 0.1;
double LATERAL_BOUNDARY_MAX = 0.5;
double SECURE_DISTANCE = 1.0;
double LATERAL_LOOKAHEAD = 2.0;
double BACK_DISTANCE = 0.3;
double EXTRA_LOOKAHEAD = 0.5;

double GOAL_RECOMPUTE_DISTANCE = 0.6;
double ORIENTATION_ERROR = 5*(M_PI/180.0);

double CONTROL_GAIN_P = 150.0;

int NORMAL_SPEED = 250;
double CHANGE_LANE_Q = 35*(M_PI/180.0);

const unsigned int FREC = 10;

//----GLOBAL VARIABLES---------------
/*
 * IMPORTANT!!!
 * ROAD_ORIENTATION, while here is declared as a global variable, should be received from a node that reads road data (e.g an image) and
 * extracts the current orientation of the road. All calculations are done based on this information, and correct navigation depends on it.
 * For integration with other autonomous driving nodes, ROAD_ORIENTATION should be set in a callback function, and not as is done in this
 * moment.
 */
float ROAD_ORIENTATION;

//----DATA STRUCTURES----------------
/*
 * carState_t is an struct containing al state data needed for navigation
 * x, y, q: pose data (position and orientation)
 * c_x, c_y: changing lane start position
 * q_r: orientation reference
 * p_s, d_s, i_s: controller state information
 * goal_set, got_pose, got_road_q: program flow flags to prevent bad behaviour
 * 
 * inLeftLane, changing_lane: booleans used for high level control of the orientation reference. 
 */
struct carState_t{
  float x; 
  float y;
  float q;
  float vel;
  
  float c_x;
  float c_y;
  
  float q_r;
  
  float p_s;
  
  bool goal_set;
  bool got_pose;
  bool got_road_q;
  
  bool inLeftLane;
  bool changing_lane;
  
  
  std_msgs::Int16 v;
  std_msgs::UInt8 w;
};

/*
 * obst_pos_t constains position data of obstacles
 */
struct obst_pos_t{
  float x;
  float y;
};

carState_t car_state;
vector<obst_pos_t> obstacles;

//Data extraction global variables
std_msgs::Float32MultiArray mov_data;
std_msgs::Byte state_data;

//----FUNCTION PROTOTYPES------------
void cb_processObstacleData(const detect_avoid::ObstacleList::ConstPtr& obst_list);
void cb_getOdometryData(const nav_msgs::Odometry::ConstPtr& pose);
void cb_getYawData(const std_msgs::Float32::ConstPtr& yaw);

bool getClassificationParams(ros::NodeHandle node);
void computeControlCmd();
void generateGoal();

int main(int argc, char **argv)
{
  //Variable initialization
  car_state.inLeftLane = false;
  car_state.got_pose = false;
  car_state.got_road_q = false;
  car_state.c_x = car_state.x;
  car_state.c_y = car_state.y;
  car_state.changing_lane = false;

 /*
  * ROS INITIALIZATION
  * With ros::init, this node starts the comunication with master.
  * ros::NodeHandle allows publication and subscription, among other features.
  * The advertise() method allows publication into the "manual_control/speed" and "steering" topics using two ros::Publisher objects.
  * ros::Rate limits the execution speed of this node
  * ros::Subscriber creates an object that allows subscription to the "obstacle_data", "odom" and "yaw" topics
  */
  ros::init(argc,argv,"movement");
  ros::NodeHandle n;
  ros::Subscriber obst_sub = n.subscribe("obstacle_data",1000, cb_processObstacleData);
  ros::Subscriber pos_sub = n.subscribe("odom",1000, cb_getOdometryData);
  ros::Subscriber yaw_sub = n.subscribe("yaw",1000, cb_getYawData);
  ros::Publisher v_pub = n.advertise<std_msgs::Int16>("manual_control/speed",1000);
  ros::Publisher w_pub = n.advertise<std_msgs::UInt8>("steering",1000);
  
  //DATA EXTRACTION PUBS
  ros::Publisher gen_state_pub = n.advertise<std_msgs::Byte>("det_state",1000);
  ros::Publisher mov_pub = n.advertise<std_msgs::Float32MultiArray>("mov",1000);
  
  ros::Rate loop_rate(FREC);
  
  ROS_INFO("[INFO]avoid -> NODE 'AVOID' STARTED");
  
 /*
  * This while loop contains the main execution part of the node, which runs while the master is live and ros::shutdown is not
  * called. With ros::spinOnce, all topics are checked for new messages, and if so, the attached callback function is called. This method
  * only does this action once, while ros::spin() does the same process indefinitely, blocking the program flow. 
  * Then, the main functions are only called if the initial information needed has beed received.
  * Also, the rate is controlled by the sleep() method, which returns true if that rate is achieved. This is used to inform the user
  * that the code is too slow or something went wrong.
  * Finally, speed and steering data is published.
  */
  while (ros::ok())
  {
    ros::spinOnce();
    if(car_state.got_pose && getClassificationParams(n))
    {
      if(!car_state.got_road_q)
      {
	car_state.got_road_q = true;
	ROAD_ORIENTATION = car_state.q;
      }
      generateGoal();
      computeControlCmd();
      ROS_INFO("[INFO]avoid -> PUBLISHING DATA: v: %d w:%d",car_state.v.data, car_state.w.data);
      v_pub.publish(car_state.v);
      w_pub.publish(car_state.w); 
      gen_state_pub.publish(state_data);
      mov_pub.publish(mov_data);
    }
    
    if(!loop_rate.sleep()) ROS_INFO("[WARN]avoid -> TIME CONDITION OF %d HZ NOT MET",FREC);
  }
  return 0;
}

void cb_processObstacleData(const detect_avoid::ObstacleList::ConstPtr& obst_list)
{
  /*
   * This callback function takes all obstacles and transforms their coordinates from polar 
   * to cartesian then stores the data in a global vector
   */
  obst_pos_t temp_obst;
  obstacles.clear();
  ROS_INFO("[DEBUG]avoid/cb_processObstacleData -> SCAN ID %d",obst_list->id);
  for (int i=0; i < obst_list->count; i++)
  {
    temp_obst.x = obst_list->d[i]*cos(obst_list->a[i]);
    temp_obst.y = obst_list->d[i]*sin(obst_list->a[i]);
    obstacles.push_back(temp_obst);
    ROS_INFO("[DEBUG]avoid/cb_processObstacleData -> OBST %d AT L[%f, %f]",i,temp_obst.x,temp_obst.y);

  }
}

void cb_getOdometryData(const nav_msgs::Odometry::ConstPtr& pose)
{
  /*
   * This callback function retrieves position data from the pose topic
   */
  car_state.x=pose->pose.pose.position.x;
  car_state.y=pose->pose.pose.position.y;
  car_state.vel=pose->twist.twist.linear.x;
}

void cb_getYawData(const std_msgs::Float32::ConstPtr& yaw)
{
  /*
   * This callback function retrieves orientation data from the yaw topic.
   * Also, sets the "got_pose" flag, as orientation is compulsory for navigation
   */
  car_state.got_pose = true;
  car_state.q = yaw->data;
}

bool getClassificationParams(ros::NodeHandle node)
{  
  bool lbm = node.getParam("LBMin", LATERAL_BOUNDARY_MIN);
  bool lbM = node.getParam("LBMax", LATERAL_BOUNDARY_MAX);
  bool ll = node.getParam("LL", LATERAL_LOOKAHEAD);
  bool bd = node.getParam("BD", BACK_DISTANCE);
  bool e = node.getParam("E", EXTRA_LOOKAHEAD);
  bool grd = node.getParam("GRD", GOAL_RECOMPUTE_DISTANCE);
  bool p = node.getParam("CP", CONTROL_GAIN_P);
  bool clq = node.getParam("CLQ", CHANGE_LANE_Q);
  
  return (lbm && lbM && ll && bd && e && grd && p && clq); 
}

void generateGoal()
{  
  /*
   * The generateGoal() procedure makes the high level control and reference generation for the control architecture.
   * -First, checks all obstacles and sets three booleans that indicates occupancy of 
   * front and lateral areas. This part is called interpretation.
   * -Then, if the previous goal has been reached, uses both occupancy information 
   * and internal state to change the value of the reference, speed and other state values. This step is called generation
   */
  
  //Interpretation
  bool front_clear = true;
  bool left_clear = true;
  bool right_clear = true;
  
  if(!car_state.changing_lane && abs(ROAD_ORIENTATION - car_state.q) < ORIENTATION_ERROR)
  { 
    for (int i=0; i < obstacles.size(); i++)
    {
      if(obstacles[i].y < -LATERAL_BOUNDARY_MIN && obstacles[i].y > -LATERAL_BOUNDARY_MAX && obstacles[i].x < LATERAL_LOOKAHEAD && obstacles[i].x > -BACK_DISTANCE)
      {
	right_clear = false;
	ROS_INFO("[INFO]avoid/generateGoal -> OBSTACLE %d [%.3f,%.3f] AT YOUR RIGHT",i,obstacles[i].x,obstacles[i].y);
      }
      else if(obstacles[i].y > LATERAL_BOUNDARY_MIN && obstacles[i].y < LATERAL_BOUNDARY_MAX && obstacles[i].x < (LATERAL_LOOKAHEAD + EXTRA_LOOKAHEAD) && obstacles[i].x > -BACK_DISTANCE)
      {
	left_clear = false;
	ROS_INFO("[INFO]avoid/generateGoal -> OBSTACLE %d [%.3f,%.3f] AT YOUR LEFT",i,obstacles[i].x,obstacles[i].y);
      }
      else if(obstacles[i].y > -LATERAL_BOUNDARY_MIN && obstacles[i].y < LATERAL_BOUNDARY_MIN && obstacles[i].x < SECURE_DISTANCE && obstacles[i].x > 0.01)
      {
	front_clear = false;
	ROS_INFO("[INFO]avoid/generateGoal -> OBSTACLE %d [%.3f,%.3f] IN FRONT OF YOU",i,obstacles[i].x,obstacles[i].y);
      }
    }
  }
  else if(abs(ROAD_ORIENTATION - car_state.q) > ORIENTATION_ERROR)
  {
    right_clear = false;
    left_clear = false;
    ROS_INFO("[INFO]avoid/generateGoal -> ORIENTATION IN LANE RESTRICTION");
  }
  ROS_INFO("[INFO]avoid/generateGoal -> STATUS: F:%d, L:%d, R:%d, IN_L:%d, CHA:%d, GS:%d, O:%.d",!front_clear,!left_clear,!right_clear,car_state.inLeftLane,car_state.changing_lane, car_state.goal_set,abs(ROAD_ORIENTATION - car_state.q)<ORIENTATION_ERROR);
  
  //Packing data for extraction

  state_data.data = ((car_state.inLeftLane << 4)|(front_clear<<3)|(right_clear<<2)|(left_clear << 1)|(car_state.changing_lane));
  
   //Generation  
  if(!car_state.goal_set)
  {
    if((car_state.inLeftLane && front_clear && !right_clear) || (!car_state.inLeftLane && front_clear))
    {
      car_state.v.data = NORMAL_SPEED;
      car_state.q_r = ROAD_ORIENTATION;
      ROS_INFO("[INFO]avoid/generateGoal -> FOLLOW ROAD");
    }
    else if(!car_state.inLeftLane && !front_clear && left_clear)
    {
      car_state.v.data = 1.2*NORMAL_SPEED;
      car_state.q_r = ROAD_ORIENTATION - CHANGE_LANE_Q;
      car_state.inLeftLane = true;
      car_state.c_x = car_state.x;
      car_state.c_y = car_state.y;
      car_state.changing_lane = true;
      ROS_INFO("[INFO]avoid/generateGoal -> SWITCH TO LEFT LANE");
    }
    else if(car_state.inLeftLane && right_clear)
    {
      car_state.v.data = 1.2*NORMAL_SPEED;
      car_state.q_r = ROAD_ORIENTATION + 0.9*CHANGE_LANE_Q;
      car_state.inLeftLane = false;
      car_state.c_x = car_state.x;
      car_state.c_y = car_state.y;
      car_state.changing_lane = true;
      ROS_INFO("[INFO]avoid/generateGoal -> SWITCH TO RIGHT LANE");
    }
    else if((car_state.inLeftLane && !front_clear && !right_clear)||(!car_state.inLeftLane && !front_clear && !left_clear)) 
    {
      car_state.v.data = 0;
      ROS_INFO("[INFO]avoid/generateGoal -> STOP");
    }
        
    car_state.goal_set = true;
  }
}

void computeControlCmd()
{
  /*
   * The generateGoal() procedure makes the low level control and reference following part of the control architecture
   * Distance is used for checking when the reference should be recomputed, while err is the input for the controller.
   * Using angles has the disadvantage of discontinuities at 0 or PI depending on the configuration, so err is processed in case its value
   * is not correct when reference and current angle are in the second and third quadrant.
   * Then, the recompute flag is set if needed for the next cycle, and lane change is marked as ended.
   * Finally,a PID-type controller is implemented. First, the internal state is calculated, then it is applied to the error and 
   * a control action is sent to the vehicle in form of steering.
   */
  float distance = sqrt(((car_state.c_y-car_state.y)*(car_state.c_y-car_state.y)) + (car_state.c_x-car_state.x)*(car_state.c_x-car_state.x)); 
  float err;
  
  if (car_state.q < -M_PI/2.0 && car_state.q_r > M_PI/2.0)
  {
    err = -2.0*M_PI + car_state.q_r - car_state.q;
  }
  else if (car_state.q > M_PI/2.0 && car_state.q_r < -M_PI/2.0)
  {
    err = 2.0*M_PI + car_state.q_r - car_state.q;
  }
  else
  {
    err = car_state.q_r - car_state.q;
  }
  
  if(car_state.goal_set && distance > GOAL_RECOMPUTE_DISTANCE)
  {
    car_state.goal_set = false;
    car_state.changing_lane = false;
  }
  
  car_state.p_s = CONTROL_GAIN_P * err;
  
  double w_unsaturated = car_state.p_s;
  int w_converted = (int)w_unsaturated + 90;
  car_state.w.data = (uint)(min(max(0,w_converted),255));
  ROS_INFO("[DEBUG]avoid/computeControlCmd -> Q=%.3f, Q_R=%.3f, D=%.3f E=%f, PID=[%.2f,%.2f,%.2f]",car_state.q, car_state.q_r, distance, err, car_state.p_s);
  
  //Packing data for extraction
  mov_data.data.clear();
  mov_data.data.push_back(car_state.q_r);
  mov_data.data.push_back(car_state.q);
}
