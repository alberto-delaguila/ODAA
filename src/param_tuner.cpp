#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

using namespace std;

const int FREC = 10;

void tuneParams(ros::NodeHandle node);
void cb_getSpeed(const nav_msgs::Odometry::ConstPtr& spd);

double speed;
int counter;

int main(int argc, char **argv)
{
  ros::init(argc,argv,"tuner");
  ros::NodeHandle n;
  ros::Subscriber spd_sub = n.subscribe("odom",1000, cb_getSpeed);
  ros::Rate loop_rate(FREC);
  
  while(ros::ok())
  {
    ros::spinOnce();
    tuneParams(n);    
    if(!loop_rate.sleep()) ROS_INFO("[WARN]param_tuner -> TIME CONDITION OF %d HZ NOT MET",FREC);
  }
  
  return 0;
}

void cb_getSpeed(const nav_msgs::Odometry::ConstPtr& spd)
{
  if (counter == 10)
  {
    speed = spd -> twist.twist.linear.x;
    ROS_INFO("[DEBUG]param_tuner/cb_getSpeed -> SPEED: %.3f", speed);
    counter = 0;
  }
  else
  {
    counter++;
  }
}

void tuneParams(ros::NodeHandle node)
{
  double t_est = 1.5;
  
  double LATERAL_BOUNDARY_MIN = 0.1; 
  double LATERAL_BOUNDARY_MAX = 0.7;
  double GOAL_RECOMPUTE_DISTANCE = 0.6;
  
  double CHANGE_LANE_Q = (-5.45*speed+36.63)*(M_PI/180);
    
  double SECURE_DISTANCE = min(max(1.0, t_est * speed),5.0);
  double LATERAL_LOOKAHEAD = max(1.5, 2.5 * t_est * speed);
  double EXTRA_LOOKAHEAD = 0.25*LATERAL_LOOKAHEAD;
  double BACK_DISTANCE = min(max(0.3, 0.1 * t_est * speed),0.8);
  ROS_INFO("[INFO]param_tuner/tuneParams -> SD=%.2f, LL=%.2f, EX=%.2f, BD=%.2f", SECURE_DISTANCE, LATERAL_LOOKAHEAD, EXTRA_LOOKAHEAD, BACK_DISTANCE);
  ROS_INFO("[INFO]param_tuner/tuneParams -> LBm=%.2f, LBM=%.2f, GRD=%.1f", LATERAL_BOUNDARY_MAX, LATERAL_BOUNDARY_MIN, GOAL_RECOMPUTE_DISTANCE);
  ROS_INFO("[INFO]param_tuner/tuneParams -> CLQ=%.3f", CHANGE_LANE_Q);
  
  node.setParam("/LBMin", LATERAL_BOUNDARY_MIN);
  node.setParam("/SD", SECURE_DISTANCE);
  node.setParam("/LBMax", LATERAL_BOUNDARY_MAX);
  node.setParam("/LL", LATERAL_LOOKAHEAD);
  node.setParam("/BD", BACK_DISTANCE);
  node.setParam("/E", EXTRA_LOOKAHEAD);
  node.setParam("/GRD", GOAL_RECOMPUTE_DISTANCE);
  node.setParam("/CLQ", CHANGE_LANE_Q);
      
}
