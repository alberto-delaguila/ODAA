#include "ros/ros.h"
#include "std_msgs/Int16.h"

void tuneParams(ros::NodeHandle n, int speed);
void cb_getSpeed(std_msgs::Int16::ConstPtr& spd);

int speed;

int main(int argc, char **argv)
{
  ros::NodeHandle n;
  ros::Subscriber pos_sub = n.subscribe("manual_control/speed",1000, cb_getSpeed);
  
  while(ros::ok())
  {
    ros::spinOnce();
    tuneParams(n, speed);
  }
  
  return 0;
}

void cb_getSpeed(std_msgs::Int16::ConstPtr& spd)
{
  speed = spd->data;
}

void tuneParams(ros::NodeHandle n, int speed)
{
  float LATERAL_BOUNDARY_MIN = 0.1; 
  float LATERAL_BOUNDARY_MAX = 0.5;
  float LATERAL_LOOKAHEAD = 2.0;
  float BACK_DISTANCE = 0.3;
  float EXTRA_LOOKAHEAD = 0.5;
  float GOAL_RECOMPUTE_DISTANCE = 0.6;
  float CHANGE_LANE_Q = 35*(M_PI/180);
  float CONTROL_GAIN_P = 150;
  
  
  n.setParam("/LBMin", LATERAL_BOUNDARY_MIN);
  n.setParam("/LBMax", LATERAL_BOUNDARY_MAX);
  n.setParam("/LL", LATERAL_LOOKAHEAD);
  n.setParam("/BD", BACK_DISTANCE);
  n.setParam("/E", EXTRA_LOOKAHEAD);
  n.setParam("/GRD", GOAL_RECOMPUTE_DISTANCE);
  n.setParam("/CP", CONTROL_GAIN_P);
  n.setParam("/CLQ", CHANGE_LANE_Q);
      
}