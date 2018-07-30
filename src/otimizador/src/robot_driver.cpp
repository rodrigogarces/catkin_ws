#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

bool have_target = false;
double turn_angle = 0;
double _clockwise = 1;


class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "cmd_vel" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  //! We will be listening to TF transforms as well
  tf::TransformListener listener_;

public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 1);
  }

  //! Drive forward a specified distance based on odometry information
  bool driveForwardOdom(double distance)
  {
    //wait for the listener to get the first message
    listener_.waitForTransform("/base_link", "/odom", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener_.lookupTransform("/base_link", "/odom", 
                              ros::Time(0), start_transform);
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to go forward at 0.25 m/s
    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = 1;
    
    ros::Rate rate(10.0);
    bool done = false;
    while (!done && nh_.ok())
    {
      //send the drive command
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("/base_link", "/odom", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }

      double pose_x = current_transform.getOrigin().getX();
      double pose_y = current_transform.getOrigin().getY();

      //std::cout << "pose_x: " << pose_x << std::endl;
      //std::cout << "pose_y: " << pose_y << std::endl;

      //see how far we've traveled
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      double dist_moved = relative_transform.getOrigin().length();

      //std::cout << "estou aqui" << std::endl; 

      if(dist_moved > distance) done = true;
    }

    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

    cmd_vel_pub_.publish(base_cmd);
    std::cout << "O robo andou!" << std::endl; 

    if (done) return true;
    return false;
  }

  bool turnOdom(bool clockwise, double radians)
  {
    std::cout << "radians: " << radians << std::endl;
    //while(radians < 0) radians += 2*M_PI;
    //while(radians > 2*M_PI) radians -= 2*M_PI;

    std::cout << "radians: " << radians << std::endl;

    //wait for the listener to get the first message
    listener_.waitForTransform("base_link", "odom", 
                               ros::Time(0), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener_.lookupTransform("base_link", "odom", 
                              ros::Time(0), start_transform);

    tf::Vector3 actual_turn_axis_ = 
        start_transform.getRotation().getAxis();
    std::cout << "current_transform: " << actual_turn_axis_.getX() << ", " << actual_turn_axis_.getY() << ", " << actual_turn_axis_.getZ() << std::endl;    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to turn at 0.75 rad/s
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
    base_cmd.angular.z = 1;
    if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;
    
    //the axis we want to be rotating by
    tf::Vector3 desired_turn_axis(0,0,1);
    if (!clockwise) desired_turn_axis = -desired_turn_axis;
    
    ros::Rate rate(10.0);
    bool done = false;
    while (!done && nh_.ok())
    {
      //send the drive command
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("base_link", "odom", 
                                  ros::Time(0), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      tf::Transform relative_transform = 
        start_transform.inverse() * current_transform;
      tf::Vector3 actual_turn_axis = 
        relative_transform.getRotation().getAxis();
      double angle_turned = relative_transform.getRotation().getAngle();
      if ( fabs(angle_turned) < 1.0e-2) continue;

      if ( actual_turn_axis.dot( desired_turn_axis ) < 0 ) 
        angle_turned = 2 * M_PI - angle_turned;

      //std::cout << "relative_transform: " << actual_turn_axis_.getX() << ", " << actual_turn_axis_.getY() << ", " << actual_turn_axis_.getZ() << std::endl; 
      //std::cout << "angle_turned: " << angle_turned << std::endl;   
      //std::cout << "radians: " << radians << std::endl;   

      if (angle_turned > radians) done = true;

    }
    if (done){
      ROS_INFO("robo girou");
      base_cmd.linear.x = base_cmd.linear.y = 0.0;
      base_cmd.angular.z = 0.0;

      cmd_vel_pub_.publish(base_cmd);
      return true;
    }
    return false;
  }
};

RobotDriver *driver;

void globalPlanCallback(const nav_msgs::Path::ConstPtr& path)
{
    int psize = path->poses.size();

    ROS_INFO("callback");

    for (int i = 2; i < psize; i++)
    {
      std::cout << "iteracao " << i << " !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

      double angleDes;
      double distance;
          //declare some variables

      tf::TransformListener listener;
      tf::TransformListener listener2;
      tf::StampedTransform poseRobot;
      geometry_msgs::PoseStamped robot_pose;
      //initialize robot angle
      double yaw_r(50);
      //try what's between {}
      try
      {
          ROS_INFO("Pose du robot ...");

          listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0) );
          listener.lookupTransform("/map", "/base_link", 
                                   ros::Time(0), poseRobot);

          //get the orientation: it is a quaternion so don't worry about the z. It doesn't mean there is a 3D thing here!
          robot_pose.pose.orientation.x = poseRobot.getRotation().getX();
          robot_pose.pose.orientation.y = poseRobot.getRotation().getY();
          robot_pose.pose.orientation.z = poseRobot.getRotation().getZ();
          robot_pose.pose.orientation.w = poseRobot.getRotation().getW();


          ros::Time current_transform = ros::Time::now(); 
          listener2.getLatestCommonTime(path->poses[i].header.frame_id, "/map", current_transform, NULL);
          //path->poses[9].header.stamp = ros::Time(0);

          std::cout << "dest_x: " << path->poses[i].pose.position.x << std::endl;
          std::cout << "dest_y: " << path->poses[i].pose.position.y << std::endl;

           //conversion
          geometry_msgs::PoseStamped goalInMap;
          listener2.transformPose("/map", path->poses[i], goalInMap);

          angleDes = tf::getYaw(path->poses[i].pose.orientation);

          /*if(path->poses[i].pose.position.x- poseRobot.getOrigin().getX() > 0){
            _clockwise = 0;
          }
          else{ _clockwise = 1;}
          */

          double dy = path->poses[i].pose.position.y - poseRobot.getOrigin().getY();
          double dx = path->poses[i].pose.position.x - poseRobot.getOrigin().getX();

          distance = sqrt(dx*dx + dy*dy);
          std::cout << "dx : " << dx << std::endl;
          std::cout << "dy : " << dy << std::endl;
          std::cout << "distance : " << distance << std::endl;
          /*std::cout << dy << std::endl;
          std::cout << dx << std::endl;
          angle = atan(dy/dx) * M_PI / 180;*/ 

          // convert the quaternion to an angle (the yaw of the robot in /map frame!)         
          yaw_r = tf::getYaw(robot_pose.pose.orientation);  
      }
      catch(tf::TransformException &ex) //if the try doesn't succeed, you fall here!
      {
          // continue
          ROS_ERROR("transfrom exception : %s",ex.what());
          //ROS_INFO("error. no robot pose!");
      }

      //angleDes is the goal angle in /map frame you get it by subscribing to goal 
      //The angleDes you should get it in a call back normally by converting the goals orientation to a yaw. Exactly as we have done to get robot's yaw

      //delta_yaw is what you want to get

      std::cout << "Angle: " << angleDes << std::endl;
      std::cout << "Yaw: " << yaw_r << std::endl;
      double delta_yaw =  angleDes - yaw_r;

      /*
      if (delta_yaw < 0)
      {
        _clockwise = 1;
        delta_yaw = (-1)*delta_yaw;
      }
      else
      {
        _clockwise = 0;
      }*/

      //and here I just make sure my angle is between minus pi and pi!
      if (delta_yaw > M_PI)
        delta_yaw -= 2*M_PI;
      if (delta_yaw <= -M_PI)
        delta_yaw += 2*M_PI;

      turn_angle = delta_yaw;   
      have_target = true;

      std::cout << "distance : " << distance << std::endl;

      _clockwise = 0;
      if (have_target)
      {
        std::cout << "tem objetivo! \n";
        bool r = driver->turnOdom(_clockwise, turn_angle);

        driver->driveForwardOdom(distance);
      }
    }
}

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  ros::Subscriber sub1 = nh.subscribe("/global_plan", 1, globalPlanCallback);

  //
  driver = new RobotDriver(nh);
  ros::spin(); 
}