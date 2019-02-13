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

    sub_odom_ = n.subscribe<nav_msgs::Odometry>("odom", 1, &LocalBehaviorNode::subRobotOdometryCallback, this );

    sub_route_ = n.subscribe<tuw_multi_robot_msgs::Route>("route", 1, &LocalBehaviorNode::subRouteCallback, this);

    viapoints_srv_ = n.advertiseService("get_viapoints", &LocalBehaviorNode::sendViapoints, this);

    semaphore_client_ = std::make_unique<stoplights_client::SemaphoreClient>(n_, robot_name_, robot_width_);

    robot_state_ = STOPPED;
    try_reach_goal_ = false; 

    while(ros::ok())
    {
        ros::spinOnce();

        if(robot_state_ != STOPPED) {
  
            updateHorizon();
            
            if(robot_state_ != WAITING) {

                generateViapoints();

                sendGoal();

                if(mbActionClient_->getState().isDone()) {
                   if(try_reach_goal_) {
                       ROS_INFO("Robot reached goal");
                       robot_state_ = STOPPED;
                   } else {
                       ROS_INFO("Robot is waiting for authorization");
                       robot_state_ = WAITING;
                   } 
                }
            }

        }

    }

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


void LocalBehaviorNode::updateHorizon()
{
    
    if(!route_.segments.empty()) {
        // Get segment that the robot has reached and check if we are leaving a segment
        robot_pose_route_segment_ = route_.segments.begin() + progress_monitor_.getProgress() + 1;

        if(std::distance(previous_robot_route_segment_,robot_pose_route_segment_) == 1) {
            semaphore_client_->advertiseLeavingSegment(previous_robot_route_segment_->segment_id, 
                                                       robot_pose_route_segment_->segment_id);
        }

        previous_robot_route_segment_ = robot_pose_route_segment_;

        for(auto it=last_accessible_route_segment_; it!=route_.segments.end(); it++) {
            
            // If no space for multiple robots on segment, ask for authorization
            if(it->width < 2 * robot_width_) {

               // If we are close enough, send the authorization request
               if(std::distance(robot_pose_route_segment_,last_accessible_route_segment_) <= 2) {

                   if(!semaphore_client_->requestAuthorization(robot_pose_route_segment_->segment_id,
                                                               it->segment_id)) 
                      break;

                } else
                    break;
 
            }
            
            last_accessible_route_segment_++;
            robot_state_ = MOVING;

        }
    } else
       ROS_ERROR("Route is empty, aborting");
       
}


void LocalBehaviorNode::generateViapoints()
{

    // Update path
    viapoints_.header = route_.header;
    viapoints_.header.stamp = ros::Time::now();
    viapoints_.poses.clear();

    geometry_msgs::Pose pose;
    
    for(auto it=route_.segments.begin(); it!=last_accessible_route_segment_; it++) {
        pose.position = it->end.position;
        pose.orientation = it->end.orientation;
        viapoints_.poses.push_back(pose);
    }
    
    // Since the router doesn't provide orientation for intermediary segment, it is computed based onsegment orientation
    if(last_accessible_route_segment_ != route_.segments.end()) {
      
        double yaw=atan2(last_accessible_route_segment_->end.position.y -
                         last_accessible_route_segment_->start.position.y,
                         last_accessible_route_segment_->end.position.x-
                         last_accessible_route_segment_->start.position.x);
        viapoints_.poses.back().orientation = tf::createQuaternionMsgFromYaw(yaw);


    } else {
        try_reach_goal_=true;
    }

    // Send goal
    if(!viapoints_.poses.empty()) 
        sendGoal();
    else
        ROS_WARN("Local behavior: no goal published, because path is empty");

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


/*void LocalBehaviorControllerNode::verifyGoalFeasibility()
{
    mbActionClient_->waitForServer();
    actionlib::SimpleClientGoalState state = mbActionClient_->getState();
    if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        // use the local costmap and see if the current goal (last_goal_sent_) is ok
        if(!goalFinder_->isGoalAttainable(last_goal_sent_) ||
            state == actionlib::SimpleClientGoalState::ABORTED ||
            state == actionlib::SimpleClientGoalState::REJECTED)
        {
            move_base_msgs::MoveBaseGoal newGoal;
            goalFinder_->findNewGoal(last_goal_sent_, newGoal.target_pose);
            mbActionClient_->sendGoal(newGoal);
            last_goal_sent_ = newGoal.target_pose;
        }
    }
    // done states = RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, or LOST
    // PENDING         = 0   # The goal has yet to be processed by the action server
    // ACTIVE          = 1   # The goal is currently being processed by the action server
    // PREEMPTED       = 2   # The goal received a cancel request after it started executing
    //                             #   and has since completed its execution (Terminal State)
    // SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
    // ABORTED         = 4   # The goal was aborted during execution by the action server due
    //                             #    to some failure (Terminal State)
    // REJECTED        = 5   # The goal was rejected by the action server without being processed,
    //                             #    because the goal was unattainable or invalid (Terminal State)
    // PREEMPTING      = 6   # The goal received a cancel request after it started executing
    //                             #    and has not yet completed execution
    // RECALLING       = 7   # The goal received a cancel request before it started executing,
    //                             #    but the action server has not yet confirmed that the goal is canceled
    // RECALLED        = 8   # The goal received a cancel request before it started executing
    //                             #    and was successfully cancelled (Terminal State)
    // LOST            = 9   # An action client can determine that a goal is LOST. This should not be
    //                             #    sent over the wire by an action server
}*/


void LocalBehaviorNode::subRouteCallback(const tuw_multi_robot_msgs::Route::ConstPtr &_route) {

    ROS_DEBUG_STREAM("New road received");
    // Reset 
    reset();
    // Initialize variables
    route_ = *_route;
    last_accessible_route_segment_ = route_.segments.begin();
    progress_monitor_.init(route_);
    // 
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
