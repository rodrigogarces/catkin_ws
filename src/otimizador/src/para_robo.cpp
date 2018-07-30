#include <vector>
#include <algorithm>
#include <ctime>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <exception>
#include <sys/time.h>
#include <fstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/ros.h>

using namespace std;

ros::Publisher pub;

bool readTargetPose = false;
bool readMapOrigin = false;
int target_pose[2];
int robot_position[2];
int origin[2];

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& ps)
{
double pose_x = ps->pose.position.x;
double pose_y = ps->pose.position.y;

	std::cout << "pose.x= " << pose_x << std::endl;
    std::cout << "pose.y= " << pose_y << std::endl;

	
  std::cout << "\nCalcula Pose: " << std::endl;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  try{
  	  listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0) );
      listener.lookupTransform("/map", "/base_link", 
                               ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    //continue;
  }

  geometry_msgs::Twist vel_msg;

  pose_x = transform.getOrigin().getX();
  pose_y = transform.getOrigin().getY();
  

  std::cout << "pose_x: " << pose_x << std::endl;
  std::cout << "pose_y: " << pose_y << std::endl;

  int grid_x = ((pose_x - origin[0]) / 0.1);
  int grid_y = ((pose_y - origin[1]) / 0.1);

  std::cout << std::endl;
  std::cout << "grid_x: " << grid_x << std::endl;
  std::cout << "grid_y: " << grid_y << std::endl;

  robot_position[0] = grid_x;
  robot_position[1] = grid_y;

  std::cout << "robot_position[0]: " << robot_position[0] << std::endl;
  std::cout << "robot_position[1]: " << robot_position[1] << std::endl;

  if (readTargetPose && readMapOrigin)
  {
  	  int destino[2] = {target_pose[0], target_pose[1]};
  	  std::cout << "destino[0]: " << destino[0] << std::endl;
      std::cout << "destino[1]: " << destino[1] << std::endl;
	  //destino[0] = 137;
	  //destino[1] = 80;

	  if (destino[0] == robot_position[0] && destino[1] == robot_position[1])
	  {
	  	std::cout << "robo chegou ao destino"<< std::endl;
	  	geometry_msgs::Twist msg;
		
		msg.linear.x = 0;
		msg.linear.y = 0;
		msg.angular.z = 0;
		
		pub.publish(msg);
	  }
	  else
	  {
	  	std::cout << "robo não chegou ao destino"<< std::endl;
	  }
  }
  else
  {
   		ROS_INFO("Pose atual do robô ou a origem do mapa foram obtidas!");
  }
}

void targetPoseCallback(const geometry_msgs::Pose2D::ConstPtr& ps)
{
	target_pose[0] = ps->x;
	target_pose[1] = ps->y;

	readTargetPose = true;
}

void mapOriginCallback(const nav_msgs::MapMetaData::ConstPtr& mmd)
{
	origin[0] = mmd->origin.position.x;
	origin[1] = mmd->origin.position.y;

	readMapOrigin = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "para_robo");

  ros::NodeHandle nh;

  ros::Subscriber sub1 = nh.subscribe("/target_pose", 1000, targetPoseCallback); 
  ros::Subscriber sub3 = nh.subscribe("/map_metadata", 1000, mapOriginCallback);
  ros::Subscriber sub2 = nh.subscribe("/vrep/pose", 1000, poseCallback);

  pub = nh.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 100);

  //sub2.shutdown();

  ros::spin();

  return 0;
}    