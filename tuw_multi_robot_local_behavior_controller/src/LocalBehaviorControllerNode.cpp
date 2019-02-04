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
#include <tuw_multi_robot_msgs/RegisterRobot.h>
#include <libssh/libssh.h>
#include <algorithm>

bool ssh_authentication(std::string ssh_server)
{
    // -------------------------------------------------------------------------
    // SSH authentification - authorized_keys have to be up to date
    // -------------------------------------------------------------------------
    int rc;
    ssh_session session = ssh_new();
    if (session == NULL)
    {
        ROS_ERROR(
            "Cannot create ssh sesssion");
        return false;
    }

    // get router url/ip from a service
    // router_hn = "tank.mesh.tpg.argentan.ifollow";
    ssh_options_set(session, SSH_OPTIONS_HOST, ssh_server.c_str());

    rc = ssh_connect(session);
    if (rc != SSH_OK)
    {
        ROS_ERROR("Error connecting.");
        return false;
    }

    rc = ssh_userauth_publickey_auto(session, NULL, NULL);

    switch(rc)
    {
        case SSH_AUTH_ERROR:
            ROS_INFO("A serious error happened");
            break;
        case SSH_AUTH_DENIED:
            ROS_INFO("The server doesn't accept that public key as an authentication token. Try another key or another method");
            break;
        case SSH_AUTH_PARTIAL:
            ROS_INFO("You've been partially authenticated, you still have to use another method");
            break;
        case SSH_AUTH_SUCCESS:
            ROS_INFO("The public key is accepted!");
            break;
        case SSH_AUTH_AGAIN:
            ROS_INFO("In nonblocking mode, you've got to call this again later");
    }
    if (rc != SSH_AUTH_SUCCESS)
    {
        ROS_ERROR(
            "Cannot register the robot. Router authentication failed. %s", ssh_get_error(session));
        // Kill the node with error message
        // exit();
        // return false;
        return false;
    }
    // auth ok, close the session and proceed
    ssh_disconnect(session);
    ssh_free(session);
    // -------------------------------------------------------------------------
    return true;
}

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

    ros::ServiceClient client = n_.serviceClient<tuw_multi_robot_msgs::RegisterRobot>("/register_robot");
    ROS_INFO_STREAM("Attempt to register.");
    client.waitForExistence();

    // Service request
    tuw_multi_robot_msgs::RegisterRobot srv;

    srv.request.id = robot_name_;

    ros::Time begin = ros::Time::now();
    // Send request
    if (!client.call(srv))
    {
            ROS_ERROR_STREAM("Unable to make a call to register robot at the registration center.");
            return;
    }
    ros::Duration round_trip = ros::Time::now() - begin;

    std::string router_id = srv.response.id;
    std::replace(router_id.begin(), router_id.end(), '_', '-');

    if (!ssh_authentication(router_id)) return;

    // Successfully register with registration center
    ROS_INFO_STREAM("Registration delay: " <<  round_trip.toSec()*1e3 << " ms");

    ros::Rate r(update_rate_);

    while ( ros::ok() ) {
        r.sleep();
        ros::spinOnce();
        publishRobotInfo();
    }
}

void LocalBehaviorControllerNode::updatePath() {

    bool try_reach_goal=false;
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

        // If no preconditions then we are trying to reach the goal
        if(i == route_.segments.size()-1) 
          try_reach_goal=true;
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

        // If robot is not trying to reach the goal, then the orientation should be computed from the segment orientation
        if(!try_reach_goal) {
            yaw=atan2(route_.segments.at(path_segment_end).end.position.y-route_.segments.at(path_segment_end).start.position.y,
                      route_.segments.at(path_segment_end).end.position.x-route_.segments.at(path_segment_end).start.position.x);
            path_.poses.back().pose.orientation=tf::createQuaternionMsgFromYaw(yaw);
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
