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

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "local_behavior_controller_node" ); /// initializes the ros node with default name
    ros::NodeHandle n;

    tuw_multi_robot_route_to_path::LocalBehaviorControllerNode ctrl ( n );

    return 0;
}

namespace tuw_multi_robot_route_to_path {
LocalBehaviorControllerNode::LocalBehaviorControllerNode ( ros::NodeHandle &n )
    : n_ ( n ), n_param_ ( "~" ) {
    robot_step_ = -1;
    route_ = tuw_multi_robot_msgs::Route();
    path_segment_end = 0;
    path_segment_start = 0;

    n_param_.param<std::string> ( "robot_name", robot_name_, "r0" );
    ROS_INFO ( "robot name = %s", robot_name_.c_str() );

    n_param_.param<double> ( "robot_radius", robot_radius_, robot_radius_ );

    n_param_.param<double> ( "robot_default_radius", robotDefaultRadius_, 0.3 );

    n_param_.param<std::string> ( "frame_id", frame_id_, "map" );

    n_param_.param<double> ( "update_rate", update_rate_, 1.0 );

    n_param_.param<bool> ( "publish_goal", publish_goal_, false);

    subCtrlState_ = n.subscribe<tuw_nav_msgs::ControllerState> ( "state_trajectory_ctrl", 1,  &LocalBehaviorControllerNode::subCtrlCb, this );
    subPose_ = n.subscribe<geometry_msgs::PoseWithCovarianceStamped> ( "pose", 1,  &LocalBehaviorControllerNode::subPoseCb, this );
    subRobotInfo_ = n.subscribe<tuw_multi_robot_msgs::RobotInfo> ( "/robot_info", 10000, &LocalBehaviorControllerNode::subRobotInfoCb, this );

    subRoute_ = n.subscribe<tuw_multi_robot_msgs::Route> ( "route", 1, &LocalBehaviorControllerNode::subRouteCb, this );

    pubRobotInfo_ = n.advertise<tuw_multi_robot_msgs::RobotInfo> ( "/robot_info", 10000 );

    // Either you publish and the goal and the navigation stack retreive the path through a service. Either you send the full path.
    if(publish_goal_) {
      ROS_INFO("Local behavior : starting goal publishing mode");
      pubGoal_ = n.advertise<geometry_msgs::PoseStamped> ( "local_goal", 1 );
      viapoints_srv_ = n.advertiseService("get_viapoints", &LocalBehaviorControllerNode::sendViapoints, this);
    } else {
      ROS_INFO("Local behavior : starting path publishing mode");
      pubPath_ = n.advertise<nav_msgs::Path>("path",1);
    }
   
    ros::Rate r(update_rate_);

    while ( ros::ok() ) {
        r.sleep();
        ros::spinOnce();
        publishRobotInfo();
    }
}

void LocalBehaviorControllerNode::updatePath() {

    bool valid = true;
    size_t last_active_segment = 0;
    // Go through all segments in the route
    for(size_t i = path_segment_start; i < route_.segments.size(); i++) {
        const tuw_multi_robot_msgs::RouteSegment &seg = route_.segments.at(i);
        // Go through all preconditions and check if they are fulfilled
        for(auto&& prec : seg.preconditions) {
            std::string other_robot_name = prec.robot_id;
            auto other_robot = robot_steps_.find(other_robot_name);
            if(other_robot == robot_steps_.end()) {
                // No robot info received for this robot
                valid = false;
            } else {
                int other_robot_process_requiered = prec.current_route_segment;
                int other_robot_process_received = robot_steps_[other_robot_name];
                if(other_robot_process_received < other_robot_process_requiered){
                    // Robot is not far enough to process further
                    valid = false;
                }
            }
        }

        // Add segments to path as long as the preconditions are fulfilled
        if(valid) {
            last_active_segment = i;
        } else {
            break;
        }
    }

    if(last_active_segment > path_segment_end) {
        path_segment_end = last_active_segment;
        path_segment_start = progress_monitor_.getProgress() + 1;
        geometry_msgs::PoseStamped pose_stamped;
        path_.header = route_.header;
        path_.header.stamp = ros::Time::now(); 
        pose_stamped.header = route_.header;
        pose_stamped.header.stamp = ros::Time::now(); 
        
        path_.poses.clear();
        double yaw{0.0};
        for(size_t i = path_segment_start; i <= path_segment_end; i++) {
            pose_stamped.pose.position = route_.segments.at(i).end.position;
            pose_stamped.pose.orientation = route_.segments.at(i).end.orientation; 
            path_.poses.push_back(pose_stamped);
        }

        // Send the path or goal to the robot
        if(!publish_goal_) {
            pubPath_.publish(path_); 
        } else {
            if(!path_.poses.empty()) 
                pubGoal_.publish(path_.poses.back());
            else 
                ROS_WARN("Local behavior : no goal published, because path is empty");
        }
    }
    
}

void LocalBehaviorControllerNode::subCtrlCb ( const tuw_nav_msgs::ControllerStateConstPtr& msg ) {
    ctrl_state_ = *msg;
}

void LocalBehaviorControllerNode::subRouteCb ( const tuw_multi_robot_msgs::Route::ConstPtr &_route ) {
    route_ = *_route;
    path_segment_end = 0;
    path_segment_start = 0;
    progress_monitor_.init(route_);
}

void LocalBehaviorControllerNode::subPoseCb ( const geometry_msgs::PoseWithCovarianceStampedConstPtr &_pose ) {

    // Transform the robot's pose in the global frame and send it to the progress monitor
    geometry_msgs::PoseStamped pose_odom_frame, pose_world_frame;
    pose_odom_frame.header=_pose->header; 
    pose_odom_frame.pose=_pose->pose.pose; 

    try {
      ros::Time now = ros::Time::now();
      if(tf_listener_.waitForTransform(_pose->header.frame_id, frame_id_, now, ros::Duration(1.0))) {
        tf_listener_.transformPose(frame_id_, pose_odom_frame, pose_world_frame);
        robot_pose_.pose = pose_world_frame.pose;
        robot_pose_.covariance = _pose->pose.covariance;
        progress_monitor_.updateProgress(tuw::Point2D(robot_pose_.pose.position.x, robot_pose_.pose.position.y) );
      } else {
        ROS_ERROR("Local behavior: transform between %s and %s is not available",_pose->header.frame_id.c_str(),frame_id_.c_str());
        return;
      } 
    } catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }

}

void LocalBehaviorControllerNode::subRobotInfoCb(const tuw_multi_robot_msgs::RobotInfo_<std::allocator<void> >::ConstPtr &_robot_info) {
    std::string other_robot_name = _robot_info->sync.robot_id;
    int other_robot_process = _robot_info->sync.current_route_segment;
    robot_steps_[other_robot_name] = other_robot_process;
}

void LocalBehaviorControllerNode::publishRobotInfo() {
    updatePath();
    robot_info_.header.stamp = ros::Time::now();
    robot_info_.header.frame_id = frame_id_;
    robot_info_.robot_name = robot_name_;
    robot_info_.pose = robot_pose_;
    robot_info_.shape = robot_info_.SHAPE_CIRCLE;
    robot_info_.shape_variables.resize ( 1 );
    robot_info_.shape_variables[0] =  robot_radius_;
    robot_info_.sync.robot_id = robot_name_;
    robot_info_.sync.current_route_segment = progress_monitor_.getProgress();
    robot_info_.mode = robot_info_.MODE_NA;
    robot_info_.status = robot_info_.STATUS_STOPPED;  // TODO
    robot_info_.good_id = robot_info_.GOOD_NA;

    pubRobotInfo_.publish ( robot_info_ );
}

bool LocalBehaviorControllerNode::sendViapoints(ifollow_nav_msgs::GetViapoints::Request  &req, ifollow_nav_msgs::GetViapoints::Response &res)
{
    geometry_msgs::PoseArray viapoints;

    for(const auto & pose_stamped : path_.poses) {
      viapoints.poses.push_back(pose_stamped.pose);
    }  
 
    res.viapoints = viapoints;
    return true;
}


}  // namespace tuw_multi_robot_route_to_path
