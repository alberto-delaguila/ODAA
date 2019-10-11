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
  double t_est = 2;
  
  double LATERAL_BOUNDARY_MIN = 0.1; 
  double LATERAL_BOUNDARY_MAX = 0.5;
  double GOAL_RECOMPUTE_DISTANCE = 1.5;
  
  double CHANGE_LANE_Q = 25*(M_PI/180);
    
  double SECURE_DISTANCE = max(0.4, t_est * speed);
  double LATERAL_LOOKAHEAD = max(1.0, 2.5 * t_est * speed);
  double EXTRA_LOOKAHEAD = 0.25*LATERAL_LOOKAHEAD;
  double BACK_DISTANCE = max(0.3, 0.25 * t_est * speed);
  ROS_INFO("[INFO]param_tuner/tuneParams -> SD=%.1f, LL=%.1f, E=%.1f, BD=%.1f", SECURE_DISTANCE, LATERAL_LOOKAHEAD, EXTRA_LOOKAHEAD, BACK_DISTANCE);
  ROS_INFO("[INFO]param_tuner/tuneParams -> LBm=%.1f, LBM=%.1f, GRD=%.1f", LATERAL_BOUNDARY_MAX, LATERAL_BOUNDARY_MIN, GOAL_RECOMPUTE_DISTANCE);

  node.setParam("/LBMin", LATERAL_BOUNDARY_MIN);
  node.setParam("/SD", SECURE_DISTANCE);
  node.setParam("/LBMax", LATERAL_BOUNDARY_MAX);
  node.setParam("/LL", LATERAL_LOOKAHEAD);
  node.setParam("/BD", BACK_DISTANCE);
  node.setParam("/E", EXTRA_LOOKAHEAD);
  node.setParam("/GRD", GOAL_RECOMPUTE_DISTANCE);
  node.setParam("/CLQ", CHANGE_LANE_Q);
      
}