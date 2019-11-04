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
#include "detect_avoid/fullDatastruct.h"


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
 * ORIENTATION_GAIN: orientation controller gain.
 * SPEED_GAIN_x: speed controller gain.
 * SPEED: speed at which the car should navigate.
 * CHANGE_LANE_Q: angle reference increment or decrement for lane changing.
 * FREC: maximum rate for main loop execution.
 */

double LATERAL_BOUNDARY_MIN;
double LATERAL_BOUNDARY_MAX;
double SECURE_DISTANCE;
double LATERAL_LOOKAHEAD;
double BACK_DISTANCE;
double EXTRA_LOOKAHEAD;
double GOAL_RECOMPUTE_DISTANCE;
double CHANGE_LANE_Q;

float SPEED = 1.66;
double SPEED_GAIN_I = 20.0;
double ORIENTATION_GAIN_P = 100.0;
double ORIENTATION_ERROR = 3*(M_PI/180);
const unsigned int FREC = 100;

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

struct carState_t{
  float x; 
  float y;
  float q;
  float vel;
  
  float c_x;
  float c_y;
  
  float q_r;
  float vel_r;
 
  float speed_state;
  
  float goal_set;
  float got_pose;
  float got_road_q;
  
  bool front;
  bool right;
  bool left;
  bool inLeftLane;
  bool changingLane;
  
  std_msgs::Int16 v;
  std_msgs::UInt8 w;
};

struct obstPos_t{
  float x;
  float y;
};

carState_t car_state;
vector<obstPos_t> obstacles;

//Data extraction global variables
detect_avoid::fullDatastruct data;

//----FUNCTION PROTOTYPES------------
void cb_processObstacleData(const detect_avoid::ObstacleList::ConstPtr& obst_list);
void cb_getOdometryData(const nav_msgs::Odometry::ConstPtr& pose);
void cb_getYawData(const std_msgs::Float32::ConstPtr& yaw);

bool getClassificationParams(ros::NodeHandle node);
void computeControlCmd();
void speedControl();
void generateGoal();

int main(int argc, char **argv)
{
  //Variable initialization
  car_state.inLeftLane = false;
  car_state.got_pose = false;
  car_state.got_road_q = false;
  car_state.c_x = car_state.x;
  car_state.c_y = car_state.y;
  car_state.changingLane = false;

  //NODE CORE ROS ELEMENTS
  ros::init(argc,argv,"movement");
  ros::NodeHandle n;
  ros::Subscriber obst_sub = n.subscribe("obstacle_data",1000, cb_processObstacleData);
  ros::Subscriber pos_sub = n.subscribe("odom",1000, cb_getOdometryData);
  ros::Subscriber yaw_sub = n.subscribe("yaw",1000, cb_getYawData);
  ros::Publisher v_pub = n.advertise<std_msgs::Int16>("manual_control/speed",1000);
  ros::Publisher w_pub = n.advertise<std_msgs::UInt8>("steering",1000);
  
  //DATA EXTRACTION PUBLISHERS
  ros::Publisher exp_pub = n.advertise<detect_avoid::fullDatastruct>("dataExtraction",1000);
  
  ros::Rate loop_rate(FREC);
  
  ROS_INFO("[INFO]movement -> NODE 'MOVEMENT' STARTED");
  

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
      speedControl();
      computeControlCmd();
      
      data.vel = car_state.vel;
      data.q = car_state.q;
      data.changingLane = car_state.changingLane;
      data.inLeftLane = car_state.inLeftLane;
      data.front = car_state.front;
      data.left = car_state.left;
      data.right = car_state.right;
      data.velr = car_state.vel_r;
      data.qr = car_state.q_r;
      data.w = car_state.w.data;
      data.v = car_state.v.data;
      data.x = car_state.x;
      data.y = car_state.y;
      exp_pub.publish(data);      
      
      ROS_INFO("[INFO]movement -> PUBLISHING DATA: v: %d w:%d",car_state.v.data, car_state.w.data);
      v_pub.publish(car_state.v);
      w_pub.publish(car_state.w);  
      
      ROS_INFO("[DEBUG]movement -> EXTRACTION DATA: V[%.3f, %.3f, %d], Q[%.3f, %.3f, %d], CL/ILL/F/L/R[%d,%d,%d,%d,%d]",data.velr,data.vel,data.v,
															  data.qr,data.q,data.w,
									      data.changingLane,data.inLeftLane,data.front,data.left,data.right);
    }
    
    if(!loop_rate.sleep()) ROS_INFO("[WARN]movement -> TIME CONDITION OF %d HZ NOT MET",FREC);
  }
  return 0;
}

void cb_processObstacleData(const detect_avoid::ObstacleList::ConstPtr& obst_list)
{
  /*
   * This callback function takes all obstacles and transforms their coordinates from polar 
   * to cartesian then stores the data in a global vector
   */
  obstPos_t temp_obst;
  obstacles.clear();
  ROS_INFO("[DEBUG]movement/cb_processObstacleData -> SCAN ID %d",obst_list->id);
  for (int i=0; i < obst_list->count; i++)
  {
    temp_obst.x = obst_list->d[i]*cos(obst_list->a[i]);
    temp_obst.y = obst_list->d[i]*sin(obst_list->a[i]);
    obstacles.push_back(temp_obst);
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
  bool lbm = node.getParam("/LBMin", LATERAL_BOUNDARY_MIN);
  bool lbM = node.getParam("/LBMax", LATERAL_BOUNDARY_MAX);
  bool ll = node.getParam("/LL", LATERAL_LOOKAHEAD);
  bool sd = node.getParam("/SD", SECURE_DISTANCE);
  bool bd = node.getParam("/BD", BACK_DISTANCE);
  bool e = node.getParam("/E", EXTRA_LOOKAHEAD);
  bool grd = node.getParam("/GRD", GOAL_RECOMPUTE_DISTANCE);
  bool clq = node.getParam("/CLQ", CHANGE_LANE_Q);
  ROS_INFO("[DEBUG]movement/getClassificationParams -> RETRIEVING OBSTACLE CLASSIFICATION PARAMETERS: %d",lbm && lbM && ll && bd && e && grd && clq && sd);
  return (lbm && lbM && ll && bd && e && grd && clq && sd); 
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
  if(!car_state.changingLane)
  {
    //Interpretation
    car_state.front = true;
    car_state.left = true;
    car_state.right = true;
    if(abs(ROAD_ORIENTATION - car_state.q) < ORIENTATION_ERROR)
    {
      for (int i=0; i < obstacles.size(); i++)
      {
	if(obstacles[i].y < -LATERAL_BOUNDARY_MIN && obstacles[i].y > -LATERAL_BOUNDARY_MAX && obstacles[i].x < LATERAL_LOOKAHEAD && obstacles[i].x > -BACK_DISTANCE)
	{
	  car_state.right = false;
	  ROS_INFO("[INFO]movement/generateGoal -> OBSTACLE %d [%.3f,%.3f] AT YOUR RIGHT",i,obstacles[i].x,obstacles[i].y);
	}
	else if(obstacles[i].y > LATERAL_BOUNDARY_MIN && obstacles[i].y < LATERAL_BOUNDARY_MAX && obstacles[i].x < (LATERAL_LOOKAHEAD + EXTRA_LOOKAHEAD) && obstacles[i].x > -BACK_DISTANCE)
	{
	  car_state.left = false;
	  ROS_INFO("[INFO]movement/generateGoal -> OBSTACLE %d [%.3f,%.3f] AT YOUR LEFT",i,obstacles[i].x,obstacles[i].y);
	}
	else if(obstacles[i].y > -LATERAL_BOUNDARY_MIN && obstacles[i].y < LATERAL_BOUNDARY_MIN && obstacles[i].x < SECURE_DISTANCE && obstacles[i].x > 0.01)
	{
	  car_state.front = false;
	  ROS_INFO("[INFO]movement/generateGoal -> OBSTACLE %d [%.3f,%.3f] IN FRONT OF YOU",i,obstacles[i].x,obstacles[i].y);
	}
      }
    }
    else
    {
      car_state.right = false;
      car_state.left = false;
    }
   }   
   ROS_INFO("[INFO]movement/generateGoal -> STATUS: GS:%d, O:%d", car_state.goal_set, (abs(ROAD_ORIENTATION - car_state.q) < ORIENTATION_ERROR));


   //Generation  
  if(!car_state.goal_set)
  {
    if((car_state.inLeftLane && car_state.front && !car_state.right) || (!car_state.inLeftLane && car_state.front))
    {
      car_state.vel_r = SPEED;
      car_state.q_r = ROAD_ORIENTATION;
      ROS_INFO("[INFO]movement/generateGoal -> FOLLOW ROAD");
    }
    else if(!car_state.inLeftLane && !car_state.front && car_state.left)
    {
      car_state.vel_r = 1.2*SPEED;
      car_state.q_r = ROAD_ORIENTATION - CHANGE_LANE_Q;
      car_state.inLeftLane = true;
      car_state.c_x = car_state.x;
      car_state.c_y = car_state.y;
      car_state.changingLane = true;
      ROS_INFO("[INFO]avoid/generateGoal -> SWITCH TO LEFT LANE");
    }
    else if(car_state.inLeftLane && car_state.right)
    {
      car_state.vel_r = 1.2*SPEED;
      car_state.q_r = 0.95*(ROAD_ORIENTATION + CHANGE_LANE_Q);
      car_state.inLeftLane = false;
      car_state.c_x = car_state.x;
      car_state.c_y = car_state.y;
      car_state.changingLane = true;
      ROS_INFO("[INFO]avoid/generateGoal -> SWITCH TO RIGHT LANE");
    }
    else if((car_state.inLeftLane && !car_state.front && !car_state.right)||(!car_state.inLeftLane && !car_state.front && !car_state.left)) 
    {
      car_state.vel_r = 0;
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
    car_state.changingLane = false;
  }
  
  int w = (int)(ORIENTATION_GAIN_P * err) + 90;
  car_state.w.data = (uint)(min(max(0,w),255));
  ROS_INFO("[DEBUG]avoid/computeControlCmd -> Q=%.3f, Q_R=%.3f, D=%.3f E=%f, A=%d",car_state.q, car_state.q_r, distance, err, car_state.w.data);
}

void speedControl()
{
  car_state.speed_state += SPEED_GAIN_I*(car_state.vel_r - car_state.vel);
  car_state.v.data = (int)max(0, (int)car_state.speed_state);
  ROS_INFO("[DEBUG]avoid/speedControl -> VR=%.3f, V=%.3f, A=%d", car_state.vel_r, car_state.vel, car_state.v.data);
}