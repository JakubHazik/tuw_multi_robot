/* Copyright (c) 2017, TU Wien
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY TU Wien ''AS IS'' AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL TU Wien BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <ros/ros.h>
#include <tuw_multi_robot_route_to_path/LocalBehaviorControllerNode.h>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "local_behavior_controller_node");  /// initializes the ros node with default name
  ros::NodeHandle n;

  tuw_multi_robot_route_to_path::LocalBehaviorControllerNode ctrl(n);

  return 0;
}

namespace tuw_multi_robot_route_to_path
{
LocalBehaviorControllerNode::LocalBehaviorControllerNode(ros::NodeHandle &n)
  : n_(n), n_param_("~")
{
  observer_ = RobotStateObserver();
  robot_step_ = -1;
  robot_route_ = tuw_multi_robot_msgs::Route();
  
  n_param_.param<std::string>("robot_name", robot_name_, "r0");
  ROS_INFO("robot name = %s", robot_name_.c_str());

  n_param_.param<double>("robot_radius", robot_radius_, robot_radius_);
  
  n_param_.param<double>("robot_default_radius", robotDefaultRadius_, 0.3);

  n_param_.param<std::string>("path_topic", topic_path_, "path");

  n_param_.param<std::string>("route_topic", topic_route_, "route");

  n_param_.param<std::string>("robotInfo_topic", topic_robot_info_, "/robot_info");
  
  n_param_.param<std::string>("pose_topic", topic_pose_, "pose");
  
  n_param_.param<std::string>("frame_map", frame_map_, "map");
  
  n_param_.param<std::string>("frame_map", frame_map_, "map");
  
  n_param_.param<double>("update_rate", update_rate_, 1.0);
  
  n_param_.param<double>("update_rate_info", update_rate_info_, 5.0);

  subPose_ = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(topic_pose_, 1,
                                                                   &LocalBehaviorControllerNode::subPoseCb, this);
  subRobotInfo_ = n.subscribe<tuw_multi_robot_msgs::RobotInfo>(topic_robot_info_, 10000, &LocalBehaviorControllerNode::subRobotInfoCb, this);

  subRoute_ = n.subscribe<tuw_multi_robot_msgs::Route>(topic_route_, 1,
                                                       &LocalBehaviorControllerNode::subRouteCb, this);
  
  pubRobotInfo_ = n.advertise<tuw_multi_robot_msgs::RobotInfo>(topic_robot_info_, 10000);
  pubPath_ = n.advertise<nav_msgs::Path>(topic_path_, 1);
  
  
  ros::Rate r(update_rate_);

  int robot_info_trigger_ = 0;
  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
    if(robot_info_trigger_ >  update_rate_ / update_rate_info_) {
        publishRobotInfo();
        robot_info_trigger_ = 0;
    }            
    robot_info_trigger_++;
  }
}

void LocalBehaviorControllerNode::publishPath(std::vector<Eigen::Vector3d> _p)
{
  nav_msgs::Path path;
  ros::Time now = ros::Time::now();
  path.header.stamp = now;
  path.header.frame_id = frame_map_;
  
  for(auto&& p : _p)
  {
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = now;
    ps.header.frame_id = frame_map_;
     
    ps.pose.position.x = p[0];
    ps.pose.position.y = p[1];
    
    Eigen::Quaternion<float> q;
    q = Eigen::AngleAxisf(p[2], Eigen::Vector3f::UnitZ());
    
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();
    
    path.poses.push_back(ps);
  }
  pubPath_.publish(path);
}

void LocalBehaviorControllerNode::updatePath()
{
  if(robot_route_.segments.size() == 0)
    return;
    
  std::vector<Eigen::Vector3d> path;
  // std::vector<PathSegment> seg_path;
  
  auto&& seg = robot_route_.segments.begin();
  seg += robot_step_;
  
  bool valid = true;
  
  // if(!robot_name_.compare("robot_0"))
  //   ROS_INFO("NEW ROBOT");

  // go through all segments in the route
  for(; valid && seg != robot_route_.segments.end(); seg++)
  {
    // go through all preconditions and check if they are fulfilled
    for(auto&& prec : seg->preconditions)
    {
      if(prec.robot_id != robot_name_ && robot_steps_[prec.robot_id] < prec.current_route_segment)
      {
        valid = false;
      }
    }
    
    // add segments to path as long as the prec. are fulfilled
    if(valid)
    {
      Eigen::Vector3d pose;
      
      double r, p, y;
      tf::Quaternion q(seg->end.orientation.x, seg->end.orientation.y, seg->end.orientation.z, seg->end.orientation.w);
      tf::Matrix3x3(q).getRPY(r, p, y);
      
      pose[0] = seg->end.position.x;
      pose[1] = seg->end.position.y;
      pose[2] = y;
      // if(!robot_name_.compare("robot_0"))
      //  ROS_INFO("%s : yaw %f", robot_name_.c_str(), y);
      
      path.emplace_back(pose);
    }
  }
  
  if(path.size() > 1)
  {
    publishPath(path);
  }
}

void LocalBehaviorControllerNode::subRouteCb(const tuw_multi_robot_msgs::Route::ConstPtr &_route)
{
  robot_route_ = *_route;
  
  if(robot_route_.segments.size() == 0)
    return;
  
  // clear when new route arrives
  robot_step_ = 0;
  
  std::vector<PathSegment> seg_path;
  
  // go through all segments in the route
  for(auto&& seg : robot_route_.segments)
  {
    PathSegment path_seg;
    path_seg.start[0] = seg.start.position.x;
    path_seg.start[1] = seg.start.position.y;
    path_seg.goal[0] = seg.end.position.x;
    path_seg.goal[1] = seg.end.position.y;
    path_seg.width = seg.width;
    
    seg_path.emplace_back(path_seg);
  }
  
  observer_.init(seg_path);
}

void LocalBehaviorControllerNode::subPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &_pose)
{
  robot_pose_ = _pose->pose;
  bool changed = true;
  
  robot_step_ = observer_.getStep(Eigen::Vector2d(_pose->pose.pose.position.x, _pose->pose.pose.position.y), changed);
}

void LocalBehaviorControllerNode::subRobotInfoCb(
    const tuw_multi_robot_msgs::RobotInfo_<std::allocator<void> >::ConstPtr &_robot_info)
{
  robot_steps_[_robot_info->sync.robot_id] = _robot_info->sync.current_route_segment;
  updatePath();
}

void LocalBehaviorControllerNode::publishRobotInfo()
{
  tuw_multi_robot_msgs::RobotInfo ri;
  ri.header.stamp = ros::Time::now();
  ri.header.frame_id = frame_map_;
  ri.robot_name = robot_name_;
  ri.pose = robot_pose_;
  ri.shape = ri.SHAPE_CIRCLE;
  ri.shape_variables.push_back(robot_radius_);
  ri.sync.robot_id = robot_name_;
  ri.sync.current_route_segment = robot_step_;
  ri.mode = ri.MODE_NA;
  ri.status = ri.STATUS_STOPPED;  // TODO
  ri.good_id = ri.GOOD_NA;

  pubRobotInfo_.publish(ri);
}
}  // namespace tuw_multi_robot_route_to_path
