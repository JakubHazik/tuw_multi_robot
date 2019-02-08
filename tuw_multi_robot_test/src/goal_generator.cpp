#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <tuw_multi_robot_msgs/RobotGoals.h>

geometry_msgs::PoseStamped pose_odom_frame;


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  pose_odom_frame.header=msg->header; 
  pose_odom_frame.pose=msg->pose.pose; 
}

double distance2d(const double & x1, const double & y1, const double & x2, const double & y2) {
  return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
}


void parseWaypointsFile(ros::NodeHandle& nh, std::vector<geometry_msgs::PoseStamped>& goals_list, const std::string & _world_frame)
{
  //csv file containing one waypoint/line in the formalism : x,y (meter),yaw (degree)
  //in frame map
  ROS_INFO("Going to parse");
  std::string goals_file;

  if (nh.getParam("file", goals_file))
      {ROS_INFO("Loaded %s as goals file", goals_file.c_str());}
  else
      {ROS_ERROR("No waypoints file specified");}

  std::ifstream data(goals_file.c_str(), std::ifstream::in);

  if (!data.is_open()){ROS_ERROR("Could not open input CSV file");}
  else {

    std::string line;

    while(std::getline(data,line))
    {
      if (line.empty()){ROS_INFO( "Empty line");}
      geometry_msgs::PoseStamped next_point;

      std::stringstream lineStream(line);
      std::string cell;
      std::getline(lineStream,cell,',');
      next_point.pose.position.x = std::stof(cell);
      std::getline(lineStream,cell,',');
      next_point.pose.position.y = std::stof(cell);
      std::getline(lineStream,cell);

      next_point.pose.orientation = tf::createQuaternionMsgFromYaw(std::stof(cell));
      next_point.header.frame_id = _world_frame;
      goals_list.push_back(next_point);
    }

  }

  ROS_INFO("Parsing completed");
}


int main(int argc, char** argv){
  ros::init(argc, argv, "goal_generator");
  ros::NodeHandle nh("~");
  std::vector<geometry_msgs::PoseStamped> goals_list;


  // ID of the robot associated with the goal generator
  std::string robot_id;
  nh.param<std::string>("robot_id", robot_id, "robot_0");
  bool use_tf;
  nh.param<bool>("use_tf", use_tf, false);
  // Frames
  std::string odom_frame;
  nh.param<std::string>("odom_frame", odom_frame, "odom");
  std::string world_frame;
  nh.param<std::string>("world_frame", world_frame, "map");
  // Run in loop or not
  bool run_once;
  nh.param<bool>("run_once", run_once, true);

  parseWaypointsFile(nh, goals_list, world_frame);

  ros::Subscriber robotPoseSub = nh.subscribe("/" + robot_id + "/odom", 1, odomCallback);
  ros::Publisher goalPub = nh.advertise<tuw_multi_robot_msgs::RobotGoals>("/labelled_goal",1); 

  // Transform
  tf::TransformListener tf_listener;
  geometry_msgs::PoseStamped pose_world_frame;

  // Create the goal structure
  tuw_multi_robot_msgs::RobotGoals labeled_goal;
  labeled_goal.robot_name=robot_id;

  ros::Rate loop_rate(2);

  unsigned goal_index=0;
  geometry_msgs::Pose goal = goals_list.at(goal_index).pose;
  labeled_goal.destinations.push_back(goal);

  bool goal_published=false;
  int n_time_publish=0;

  ros::spinOnce();

  while(ros::ok()) {

    ros::spinOnce();

    if(use_tf) {
      try {
        tf_listener.waitForTransform(pose_odom_frame.header.frame_id, world_frame,
                                  ros::Time::now(), ros::Duration(1.0));

        pose_world_frame.header.stamp=ros::Time::now(); 
        tf_listener.transformPose(world_frame, pose_odom_frame, pose_world_frame);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
    } else {
      if(pose_odom_frame.header.frame_id != world_frame) {
        ROS_ERROR("Odometry frame (\"%s\") is not equal to the world frame (\"%s\"), please enable use_tf or publish in the right frame",odom_frame.c_str(),world_frame.c_str());
        break;
      }
    }
    
    ROS_ERROR("%f",distance2d(goal.position.x,goal.position.y,pose_odom_frame.pose.position.x,pose_odom_frame.pose.position.y));

    if(distance2d(goal.position.x,goal.position.y,pose_odom_frame.pose.position.x,pose_odom_frame.pose.position.y) < 1.0) {
      ros::Duration(2.0).sleep();
      goal_index++;
      // If we reach the end of the list 
      if(goal_index>=goals_list.size()) {
        if(run_once)
          break; 
        else
          goal_index=0;
      }
      goal = goals_list.at(goal_index).pose;
      labeled_goal.destinations.at(0) = goal;
      goal_published=false;
      n_time_publish=0;
    }
   
    // Republish several times to be sure that the goal is sent   
    if(!goal_published || n_time_publish<5) {
      goalPub.publish(labeled_goal);
      ROS_ERROR("Goal published");
      goal_published=true;
      n_time_publish++;
    }

    loop_rate.sleep();
 
  }

  return 0;
}
