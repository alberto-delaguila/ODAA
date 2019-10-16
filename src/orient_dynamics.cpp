#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "detect_avoid/fullDatastruct.h"
using namespace std;


std_msgs::Int16 SPEED;
const float CHANGE_LANE_Q = 35*(M_PI/180);
const unsigned P = 150;
const unsigned int FREC = 10;

detect_avoid::fullDatastruct q_data;

void cb_getYawData(const std_msgs::Float32::ConstPtr& yaw);
void cb_getSpdData(const nav_msgs::Odometry::ConstPtr& odom);
std_msgs::UInt8 computeControlCmd();
float generateInput(double t0);

bool close_loop = true;
bool got_yaw = false;
float offset;

float q;
float qr;
float spd;

int main(int argc, char **argv)
{
  ros::init(argc,argv,"dynamics");
  ros::NodeHandle n;
  ros::Subscriber yaw_sub = n.subscribe("yaw",1000, cb_getYawData);
  ros::Subscriber v_sub = n.subscribe("odom",1000, cb_getSpdData);
  ros::Publisher v_pub = n.advertise<std_msgs::Int16>("manual_control/speed",1000);
  ros::Publisher w_pub = n.advertise<std_msgs::UInt8>("steering",1000);
  ros::Publisher data_pub = n.advertise<detect_avoid::fullDatastruct>("dataExtraction",1000);

  ros::Rate loop_rate(FREC);
  
  SPEED.data = 862;
  
  while(!got_yaw)
  {
    if(!close_loop)
    {
      ROS_INFO("WAITING FOR YAW. OPEN LOOP MODE");
      ros::spinOnce();
      offset = 0;
      loop_rate.sleep();
    }
    else
    {
      ROS_INFO("WAITING FOR YAW. CLOSED LOOP MODE");
      ros::spinOnce();
      offset = q;
      loop_rate.sleep();
    }
  }
  
   
  double start_time = ros::Time::now().toSec();
  v_pub.publish(SPEED);
  while (ros::ok())
  {
    ros::spinOnce();
    qr = generateInput(start_time);
    std_msgs::UInt8 action = computeControlCmd();
    
    q_data.q = q;
    q_data.qr = qr;
    q_data.w = action.data;
    q_data.v = SPEED.data;
    q_data.vel = spd;
    
    w_pub.publish(action);
    data_pub.publish(q_data);

    
    ROS_INFO("DATA PUBLISHED: Q_R %.2f, Q %.2f, W %d",qr,q,action);
    loop_rate.sleep();
  }
  return 0;
}

void cb_getYawData(const std_msgs::Float32::ConstPtr& yaw)
{
  got_yaw = true;
  q = yaw->data;
}

void cb_getSpdData(const nav_msgs::Odometry::ConstPtr& odom)
{
  spd = odom->twist.twist.linear.x;
}

float generateInput(double t0)
{
  double elapsed = (ros::Time::now().toSec() - t0);
  if(elapsed < 1.0)
  {
    ROS_INFO("PRIMERA PARTE");
    return offset;
  }
  else
  {
    ROS_INFO("SEGUNDA PARTE");
    return offset + CHANGE_LANE_Q;
  }
}

std_msgs::UInt8 computeControlCmd()
{ 
  float input;
  std_msgs::UInt8 res;
  if(!close_loop)
  {
    input = qr;
  }
  else
  {
    float error = qr - q;
    input = error;
  }
  int w_converted  = (P*input + 90);
  res.data = (min(max(0,w_converted),255));
  return res;
}
