#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "detect_avoid/fullDatastruct.h"
using namespace std;

const float INC = 1;
const unsigned I = 681;
const unsigned int FREC = 10;

detect_avoid::fullDatastruct v_data;

void cb_getSpdData(const nav_msgs::Odometry::ConstPtr& odom);
void computeControlCmd();
void generateInput(double t0);

bool close_loop = true;
bool got_spd = false;

float v;
float vr;
std_msgs::Int16 vel;

int main(int argc, char **argv)
{
  ros::init(argc,argv,"dynamics");
  ros::NodeHandle n;
  ros::Subscriber yaw_sub = n.subscribe("odom",1000, cb_getSpdData);
  ros::Publisher v_pub = n.advertise<std_msgs::Int16>("manual_control/speed",1000);
  ros::Publisher data_pub = n.advertise<detect_avoid::fullDatastruct>("dataExtraction",1000);

  ros::Rate loop_rate(FREC);
   
  while(!got_spd)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
   
  double start_time = ros::Time::now().toSec();

  while (ros::ok())
  {
    ros::spinOnce();
    generateInput(start_time);
    computeControlCmd();
    
    v_data.vel = v;
    v_data.vel = vr;
    v_data.v = vel.data;
    
    v_pub.publish(vel);
    data_pub.publish(v_data);

    
    ROS_INFO("DATA PUBLISHED: V_R %.2f, V %.2f, A %d",vr,v,vel.data);
    loop_rate.sleep();
  }
  return 0;
}

void cb_getSpdData(const nav_msgs::Odometry::ConstPtr& odom)
{
  got_spd = true;
  v = odom->twist.twist.linear.x;
}

void generateInput(double t0)
{
  double elapsed = (ros::Time::now().toSec() - t0);
  if(elapsed < 1.0)
  {
    ROS_INFO("PRIMERA PARTE");
    vr = 0;
  }
  else
  {
    ROS_INFO("SEGUNDA PARTE");
    vr = INC;
  }
}

void computeControlCmd()
{ 
  if(!close_loop)
  {
    vel.data = vr;
  }
  else
  {
    float error = vr - v;
    vel.data += I*error;
  }
}
