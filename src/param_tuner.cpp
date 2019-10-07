#include "ros/ros.h"
#include "std_msgs/Int16.h"

void tuneParams(ros::NodeHandle node, int speed);
void cb_getSpeed(const std_msgs::Int16::ConstPtr& spd);

int speed;

int main(int argc, char **argv)
{
  ros::init(argc,argv,"tuner");
  ros::NodeHandle n;
  ros::Subscriber spd_sub = n.subscribe("/manual_control/speed",1000, cb_getSpeed);
  
  while(ros::ok())
  {
    ros::spinOnce();
    tuneParams(n, speed);
  }
  
  return 0;
}

void cb_getSpeed(const std_msgs::Int16::ConstPtr& spd)
{
  speed = spd->data;
}

void tuneParams(ros::NodeHandle node, int speed)
{
  double LATERAL_BOUNDARY_MIN = 0.1; 
  double LATERAL_BOUNDARY_MAX = 0.5;
  double LATERAL_LOOKAHEAD = 2.0;
  double BACK_DISTANCE = 0.3;
  double EXTRA_LOOKAHEAD = 0.5;
  double GOAL_RECOMPUTE_DISTANCE = 0.6;
  double CHANGE_LANE_Q = 35*(M_PI/180);
  double CONTROL_GAIN_P = 150.0;
  
  
  node.setParam("/LBMin", LATERAL_BOUNDARY_MIN);
  node.setParam("/LBMax", LATERAL_BOUNDARY_MAX);
  node.setParam("/LL", LATERAL_LOOKAHEAD);
  node.setParam("/BD", BACK_DISTANCE);
  node.setParam("/E", EXTRA_LOOKAHEAD);
  node.setParam("/GRD", GOAL_RECOMPUTE_DISTANCE);
  node.setParam("/CP", CONTROL_GAIN_P);
  node.setParam("/CLQ", CHANGE_LANE_Q);
      
}