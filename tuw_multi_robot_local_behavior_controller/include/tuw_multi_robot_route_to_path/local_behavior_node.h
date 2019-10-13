#ifndef LOCAL_BEHAVIOR_NODE_H
#define LOCAL_BEHAVIOR_NODE_H

#include <memory>

// ROS
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>

#include <tuw_multi_robot_msgs/Route.h>
#include <tuw_multi_robot_route_to_path/RouteProgressMonitor.h>
#include <ifollow_nav_msgs/GetViapoints.h>
#include <tuw_multi_robot_route_to_path/GoalFinder.h>
#include <stoplights_client/semaphore_client.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef std::shared_ptr<MoveBaseClient> MoveBaseClientPtr;

namespace tuw_multi_robot_route_to_path
{

    enum RobotState { STOPPED, WAITING, MOVING };

    class LocalBehaviorNode
    {
      public:
        LocalBehaviorNode(ros::NodeHandle &n);

        // ROS:
        ros::NodeHandle n_;        ///< Node handler to the root node
        ros::NodeHandle n_param_;  ///< Node handler to the current node
        std::unique_ptr<ros::Rate> rate_;

      private: 
        // Methods
        void updateHorizon(); 
        void generateViapoints();
        void verifyGoalFeasibility();
        bool sendViapoints(ifollow_nav_msgs::GetViapoints::Request  &req, ifollow_nav_msgs::GetViapoints::Response &res);
        bool sendGoal();
        void reset();
        // Callbacks
        void subRobotOdometryCallback(const nav_msgs::Odometry::ConstPtr& _odom);
        void subRouteCallback(const tuw_multi_robot_msgs::Route::ConstPtr& _route);
        void checkSegmentTransition();
        bool checkRestrictedSegment( const tuw_multi_robot_msgs::RouteSegment & _seg );
        geometry_msgs::Quaternion computeSegmentOrientation( const tuw_multi_robot_msgs::RouteSegment & _seg );

        // ROS parameters
        double update_rate_;
        int dist_to_restricted_segment_;
        std::string frame_id_;
        std::string robot_name_;
        double robot_width_;

        // Components
        MoveBaseClientPtr mbActionClient_;
        ros::ServiceServer viapoints_srv_;
        tf::TransformListener tf_listener_;
        goal_finder::GoalFinderPtr goalFinder_;
        std::unique_ptr<stoplights_client::SemaphoreClient> semaphore_client_;
        tuw::RouteProgressMonitor progress_monitor_;
        ros::Subscriber sub_route_;
        ros::Subscriber sub_odom_;

        // State variables
        RobotState robot_state_;
        bool try_reach_goal_;
        geometry_msgs::PoseWithCovariance robot_pose_;
        tuw_multi_robot_msgs::Route route_;
        geometry_msgs::PoseArray viapoints_;
        geometry_msgs::PoseStamped goal_;
        geometry_msgs::PoseStamped last_goal_sent_;
        std::vector<tuw_multi_robot_msgs::RouteSegment>::iterator robot_pose_route_segment_;
        std::vector<tuw_multi_robot_msgs::RouteSegment>::iterator previous_robot_route_segment_;
        std::vector<tuw_multi_robot_msgs::RouteSegment>::iterator last_accessible_route_segment_;

    };

}  // namespace tuw_multi_robot_route_to_path

#endif  // LOCAL_BEHAVIOR_NODE_H
