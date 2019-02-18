#include <ros/ros.h>
#include <tuw_multi_robot_route_to_path/local_behavior_node.h>
#include <tf/transform_datatypes.h>
#include <tuw_multi_robot_msgs/RegisterRobot.h>
#include <libssh/libssh.h>
#include <algorithm>
#include <pose_cov_ops/pose_cov_ops.h>

int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "local_behavior_node" ); /// initializes the ros node with default name
    ros::NodeHandle n;

    tuw_multi_robot_route_to_path::LocalBehaviorNode ctrl ( n );

    return 0;
}


namespace tuw_multi_robot_route_to_path {


LocalBehaviorNode::LocalBehaviorNode(ros::NodeHandle &n)
    : n_(n)
    , n_param_("~")
    , mbActionClient_(new MoveBaseClient("move_base", true))
    , goalFinder_(new goal_finder::GoalFinder())
{
    route_ = tuw_multi_robot_msgs::Route();

    n_param_.param<std::string>("robot_name", robot_name_, "ilogistics_2_xxxx");
    ROS_INFO("robot id = %s", robot_name_.c_str());

    n_param_.param<double>("robot_width", robot_width_, 0.76);

    n_param_.param<std::string>("frame_id", frame_id_,"map");

    n_param_.param<double>("update_rate", update_rate_, 1.0);

    if(update_rate_ < 0.99 || update_rate_ > 60.0) 
        throw "Out of range update rate (should be included between 1.0 and 60.0 Hz)";

    n_param_.param<int>("dist_to_restricted_segment", dist_to_restricted_segment_, 2 ); 
    
    if(dist_to_restricted_segment_ < 1)  
        throw "Distance to segment should be bigger than 1";

    sub_odom_ = n.subscribe<nav_msgs::Odometry>("odom", 1, &LocalBehaviorNode::subRobotOdometryCallback, this );

    sub_route_ = n.subscribe<tuw_multi_robot_msgs::Route>("route", 1, &LocalBehaviorNode::subRouteCallback, this);

    viapoints_srv_ = n.advertiseService("get_viapoints", &LocalBehaviorNode::sendViapoints, this);

    semaphore_client_ = std::make_unique<stoplights_client::SemaphoreClient>(n_, robot_name_, robot_width_);

    robot_state_ = STOPPED;
    try_reach_goal_ = false; 


    rate_ = std::make_unique<ros::Rate>(update_rate_);

    while(ros::ok())
    {

        rate_->sleep();
         
        ros::spinOnce();

        switch(robot_state_) {
            // If the robot is stopped, just update the callback 
            case STOPPED:
                break;
            // If the robot is waiting authorization for a restricted segment
            case WAITING:
                checkSegmentTransition();
                updateHorizon();
                break;
            // If the robot is moving to its goal
            case MOVING:
                checkSegmentTransition();
                updateHorizon();
                generateViapoints();
                sendGoal();
                
                // Check if move_base has reached goal 
                if(mbActionClient_->getState().isDone()) {
                   if(try_reach_goal_) {
                       // If we are reaching destination, then robot will be stopped
                       ROS_INFO("Robot reached goal");
                       robot_state_ = STOPPED;
                   } else {
                       // If we are not reaching destination, then the robot will wait for authorization
                       ROS_INFO("Robot is waiting for authorization");
                       robot_state_ = WAITING;
                   } 
                }
                break;

        }

    }

}


void LocalBehaviorNode::checkSegmentTransition() {

    if(!route_.segments.empty()) {
        // Get segment that the robot has reached and check if we are leaving it
        robot_pose_route_segment_ = route_.segments.begin() + progress_monitor_.getProgress() + 1;
 
        // If the distance between the two segments is one, then we are currently leaving a segment
        if(std::distance(previous_robot_route_segment_,robot_pose_route_segment_) == 1) {
            // Advertise the server that we are freeing the segment
            semaphore_client_->advertiseLeavingSegment(previous_robot_route_segment_->segment_id, 
                                                       robot_pose_route_segment_->segment_id);
        }
 
        previous_robot_route_segment_ = robot_pose_route_segment_;

    } else
       ROS_ERROR_STREAM(ros::this_node::getName() << " : route is empty, aborting");
 
}


void LocalBehaviorNode::updateHorizon()
{
    
    if(!route_.segments.empty()) {

        for(auto it=last_accessible_route_segment_; it!=route_.segments.end(); it++) {
            
            // Check if the segment is restricted
            if( checkRestrictedSegment(*last_accessible_route_segment_) ) {

               // Ask authorization for the segment at the requested distance
               if(std::distance(robot_pose_route_segment_,last_accessible_route_segment_) <= dist_to_restricted_segment_ + 1) {
                   
                   if(it != route_.segments.begin()) {
                       // Request authorization to the server 
                       if(!semaphore_client_->requestAuthorization((it-1)->segment_id,
                                                                    it->segment_id)) 
                          break;

                   } else 
                       ROS_ERROR_STREAM(ros::this_node::getName() << " : can't request access for the first segment");

                } else
                    break;
 
            }
           
            // If the segment is either large enough or we granted access to the segment, 
            // we increment the iterator 
            last_accessible_route_segment_++;
            // If the horizon has been updated then the robot is moving
            robot_state_ = MOVING;

        }
    } else
       ROS_ERROR_STREAM(ros::this_node::getName() << " : route is empty, aborting");
       
}


void LocalBehaviorNode::generateViapoints()
{

    // Update path
    viapoints_.header = route_.header;
    viapoints_.header.stamp = ros::Time::now();
    viapoints_.poses.clear();

    geometry_msgs::Pose pose;
  
    if(last_accessible_route_segment_ == route_.segments.end()) {

        // If we are reaching the destination, provide full path
        for(auto it=route_.segments.begin(); it!=route_.segments.end(); it++) {
            
            pose.position = it->end.position;
   
            if(it != route_.segments.end()-1) 
                pose.orientation = computeSegmentOrientation(*it);
            else
                pose.orientation = it->end.orientation;

            viapoints_.poses.push_back(pose);
        }

    } else if (std::distance(robot_pose_route_segment_, last_accessible_route_segment_) < dist_to_restricted_segment_) {

        // If distance to last segment lower than maximal dist, don't crope        
        for(auto it=route_.segments.begin(); it!=last_accessible_route_segment_; it++) {
            pose.position = it->end.position;
            pose.orientation = computeSegmentOrientation(*it);
            viapoints_.poses.push_back(pose);
        }

    } else {

        // else stop two segments before
        for(auto it=route_.segments.begin(); it!=last_accessible_route_segment_ - dist_to_restricted_segment_; it++) {
            pose.position = it->end.position;
            pose.orientation = computeSegmentOrientation(*it);
            viapoints_.poses.push_back(pose);
        }

    }
 
    
    if(last_accessible_route_segment_ == route_.segments.end()) 
        try_reach_goal_=true;

}


bool LocalBehaviorNode::sendGoal() {

    if(viapoints_.poses.empty()) {
        ROS_ERROR("No viapoints provided");
        return false;
    }    

    // New goal candidate
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header = viapoints_.header;
    goal.target_pose.pose = viapoints_.poses.back();

    geometry_msgs::PoseStamped last_goal_sent_correct_frame = last_goal_sent_;

    // If last goal sent was empty
    if ( last_goal_sent_.header.frame_id != "" ) {
        // If frame id of current and previous goal is not the same, transform
        if( goal.target_pose.header.frame_id != last_goal_sent_.header.frame_id )
        {
            if(tf_listener_.waitForTransform(goal.target_pose.header.frame_id, last_goal_sent_.header.frame_id, ros::Time(0), ros::Duration(1.0)))
            {
                tf_listener_.transformPose(goal.target_pose.header.frame_id, last_goal_sent_, last_goal_sent_correct_frame);
            }
            else
            {
                ROS_WARN_STREAM("Transform between " << goal.target_pose.header.frame_id << " and " << last_goal_sent_.header.frame_id << " not available!");
                return false;
            }
        }
        
        geometry_msgs::Pose diff;
        pose_cov_ops::inverseCompose(goal.target_pose.pose, last_goal_sent_correct_frame.pose, diff);

        // Send a goal only if it is a new one
        if (diff.position.x > 1e-6 ||
            diff.position.y > 1e-6 ||
            diff.orientation.x > 1e-6 ||
            diff.orientation.y > 1e-6 ||
            diff.orientation.z > 1e-6 ||
            (diff.orientation.w - 1.0) > 1e-6 )
        {
            ROS_ERROR("Send new goal");
            // Wait for move base action server
            mbActionClient_->waitForServer();
            mbActionClient_->sendGoal(goal);
            last_goal_sent_.header = viapoints_.header;
            last_goal_sent_.pose = viapoints_.poses.back();
        }

    } else {
        // Send goal to move_base
        mbActionClient_->waitForServer();
        mbActionClient_->sendGoal(goal);
        // Update the last goal sent
        last_goal_sent_.header = viapoints_.header;
        last_goal_sent_.pose = viapoints_.poses.back();
    } 

    return true;

}


bool LocalBehaviorNode::checkRestrictedSegment( const tuw_multi_robot_msgs::RouteSegment & _seg ) 
{
    // Restriction criterion is a segment without enough space for two robots to cross
    if(_seg.width < 2 * robot_width_)
        return true;
    else
        return false;

}


geometry_msgs::Quaternion LocalBehaviorNode::computeSegmentOrientation( const tuw_multi_robot_msgs::RouteSegment & _seg ) 
{

    double yaw = atan2(_seg.end.position.y - _seg.start.position.y,
                       _seg.end.position.x - _seg.start.position.x);

    return tf::createQuaternionMsgFromYaw(yaw);

}


void LocalBehaviorNode::reset()
{

    ROS_DEBUG_STREAM("Resetting local behavior");
    route_ = tuw_multi_robot_msgs::Route(); 
    viapoints_ = geometry_msgs::PoseArray();
    last_goal_sent_ = geometry_msgs::PoseStamped();
    robot_pose_route_segment_ = route_.segments.begin();
    previous_robot_route_segment_ = route_.segments.begin();
    last_accessible_route_segment_ = route_.segments.begin();
    robot_state_ = STOPPED;
    try_reach_goal_ = false;

}


void LocalBehaviorNode::subRouteCallback(const tuw_multi_robot_msgs::Route::ConstPtr &_route) {

    ROS_DEBUG_STREAM("New road received");
    // Reset 
    reset();
    // Initialize variables
    route_ = *_route;
    last_accessible_route_segment_ = route_.segments.begin() + 1;
    progress_monitor_.init(route_);
    robot_pose_route_segment_ = route_.segments.begin() + progress_monitor_.getProgress() + 1;
    previous_robot_route_segment_ = route_.segments.begin() + progress_monitor_.getProgress() + 1;
    ROS_DEBUG_STREAM("Robot enter moving state");
    robot_state_ = WAITING;

}


void LocalBehaviorNode::subRobotOdometryCallback(const nav_msgs::Odometry::ConstPtr &_odom) {

    // Transform the robot's pose in the global frame and send it to the progress monitor
    geometry_msgs::PoseStamped pose_odom_frame, pose_world_frame;
    pose_odom_frame.header=_odom->header;
    pose_odom_frame.pose=_odom->pose.pose;

    try {
        ros::Time now = ros::Time::now();
        if(tf_listener_.waitForTransform(_odom->header.frame_id, frame_id_, now, ros::Duration(1.0))) {
            tf_listener_.transformPose(frame_id_, pose_odom_frame, pose_world_frame);
            robot_pose_.pose = pose_world_frame.pose;
            robot_pose_.covariance = _odom->pose.covariance;
            progress_monitor_.updateProgress(tuw::Point2D(robot_pose_.pose.position.x, robot_pose_.pose.position.y) );
        } else {
            ROS_ERROR("Local behavior: transform between %s and %s is not available",_odom->header.frame_id.c_str(),frame_id_.c_str());
            return;
        }
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

} 


bool LocalBehaviorNode::sendViapoints(ifollow_nav_msgs::GetViapoints::Request  &req, ifollow_nav_msgs::GetViapoints::Response &res)
{

    res.viapoints = viapoints_;
    return true;

}



}  // namespace tuw_multi_robot_route_to_path
