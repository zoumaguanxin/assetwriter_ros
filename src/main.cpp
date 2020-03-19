#include <iostream>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include <tf/tf.h>
#include <cmath>

#include <Eigen/Dense>

using namespace std;

std::string filedir="../data/odomSyclaser";

ofstream file;

Eigen::Vector3f t_last;
double theta_last;

bool start=false;

double t_update=0.1;
double angular_update=0.0872;


std::ofstream& operator<<(std::ofstream& out, nav_msgs::OdometryConstPtr odomPtr)
{
  std::string odomHeader="odomxyz_xyzw:";
  out<<odomHeader<<" ";
  out<<odomPtr->pose.pose.position.x<<" "<<odomPtr->pose.pose.position.y<<" "<<odomPtr->pose.pose.position.z<<" ";
  out<<odomPtr->pose.pose.orientation.x<<" "<<odomPtr->pose.pose.orientation.y<<" "<<odomPtr->pose.pose.orientation.z<<" "<<odomPtr->pose.pose.orientation.w;  
  out<<std::endl;
  return out;
}


std::ofstream& operator<<(std::ofstream& out, sensor_msgs::LaserScanConstPtr scanPtr)
{
  std::string laserType="laser_fr_Amin_incre_max_Rmax_ranges:";
  out<<scanPtr->header.frame_id<<" ";
  out<<scanPtr->angle_min<<" "<<scanPtr->angle_increment<<" "<<scanPtr->angle_max<<" "<<scanPtr->range_max<<" "<<scanPtr->ranges.size()<<" ";
  for(float range:scanPtr->ranges)
  {
    out<<range<<" ";
  }
  out<<std::endl;
  return out;
}





void  callback(const nav_msgs::OdometryConstPtr odom, const sensor_msgs::LaserScanConstPtr scanPtr )
{
  Eigen::Vector3f t_current,t_diff;
  double theta_current,theta_diff;
  
  
  tf::Quaternion q(odom->pose.pose.orientation.x,odom->pose.pose.orientation.y,odom->pose.pose.orientation.z,odom->pose.pose.orientation.w); 
  
  if(!start)
  {
    
    t_current(0)=odom->pose.pose.position.x;
    t_current(1)=odom->pose.pose.position.y;
    t_current(2)=odom->pose.pose.position.z;    
    theta_current=tf::getYaw(q);   
    
  }
  else
  {
    t_current(0)=odom->pose.pose.position.x;
    t_current(1)=odom->pose.pose.position.y;
    t_current(2)=odom->pose.pose.position.z;
    t_diff=t_current-t_last;         
    tf::getYaw(q);
    theta_current=tf::getYaw(q);
    theta_diff=theta_current-theta_last;
    
  }
  std::cout<<std::fabs(t_diff(0))<<" "<<std::fabs(t_diff(1))<<" "<<std::fabs(theta_diff)<<" "<<std::endl;
  bool flag=std::fabs(t_diff(0))>t_update||std::fabs(t_diff(1))>t_update||(!start)||std::fabs(theta_diff)>angular_update?true:false;  
  start=true;
  if(flag)
  {  
      t_last=t_current;
      theta_last=theta_current;
      ROS_INFO("begin to record data");
      if(file.is_open())
      {
	if(file.good())
	{
	file<<odom;
	file<<scanPtr;
	}
	else
	{
	ROS_WARN("failed to open file in %s", filedir.c_str());
	}
      }
      else
      {
	file.open(filedir.c_str(), ios_base::out);
	if(file.good())
	{
		file<<odom;
		file<<scanPtr;
	}
	else
	{
	  ROS_WARN("failed to open file in %s", filedir.c_str());
	}
      }
  }
  
 
  
  
}

int main(int argc, char **argv) {
    
    
    ros::init(argc,argv,"assertwriter");
    ros::NodeHandle nh;   
    
    
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "/scan",1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh,"/odom",1);
    
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::LaserScan> odom_laser_approcSync_policy;
    
    
    
    message_filters::Synchronizer<odom_laser_approcSync_policy> *xx ;
    xx=  new message_filters::Synchronizer<odom_laser_approcSync_policy>(odom_laser_approcSync_policy(10), odom_sub, scan_sub);
    
    //note the sequence of template must be consistent with the class of parameters of callback function 
    //message_filters::TimeSynchronizer<nav_msgs::Odometry,sensor_msgs::LaserScan > sync(odom_sub, scan_sub,50);
    
    
    ROS_INFO("register callback function");
    xx->registerCallback(boost::bind(&callback, _1, _2));
    
    ros::spin();
    
    return 0;
}
