/*
 * Copyright (c) 2017, <copyright holder> <email>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef ROUTER_NODE_H
#define ROUTER_NODE_H

//ROS
#include <ros/ros.h>
#include <tuw_multi_robot_msgs/RobotGoalsArray.h>
#include <tuw_multi_robot_msgs/RobotGoals.h>
#include <tuw_global_router/robot_info.h>
#include <nav_msgs/Odometry.h>
#include <tuw_multi_robot_msgs/Graph.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tuw_multi_robot_msgs/RouterStatus.h>
#include <dynamic_reconfigure/server.h>
#include <tuw_multi_robot_router/routerConfig.h>

#include <tuw_global_router/router.h>
#include <tuw_global_router/mrr_utils.h>
#include <opencv/cv.hpp>
#include <mutex>
#include <tuw_multi_robot_msgs/RegisterRobot.h>
// #include <tuw_multi_robot_msgs/UnregisterRobot.h>
//TODO disable got_map if not used

namespace multi_robot_router
{
class Router_Node : Router
{
public:
    /**
     * @brief Construct a new Router_Node object
     * @param n the nodehandle to register publishers and subscribers
     */
    Router_Node ( ros::NodeHandle &n );

    /**
     * @brief publishes an empty RoutingTable
     */
    void publishEmpty();

    /**
     * @brief publishes a RoutingTable
     */
    void publish();

    /**
     * @brief Monitors the execution
     * Check status of every robots and decide if execution is finished or not
     */
    void monitorExecution();

    /**
     * @brief used to update the nodes timeout to latch topics
     * @param secs the seconds passed since the last update
     */
    void updateTimeout ( const float _secs );

    /**
     * @brief run the multi_robot_router
     */
    void plan();
    ros::NodeHandle n_;       ///< Node handler to the root node
    ros::NodeHandle n_param_; ///< Node handler to the current node

private:
    // Callback for the dynamic reconfigure
    void parametersCallback ( tuw_multi_robot_router::routerConfig &config, uint32_t level );
    // Callback to get the graph data
    void graphCallback ( const tuw_multi_robot_msgs::Graph &msg );
    // Callback to receive an array of goals
    void goalsCallback ( const tuw_multi_robot_msgs::RobotGoalsArray &_goals );
    // Callback to get the map
    void mapCallback ( const nav_msgs::OccupancyGrid &_map );
    // Callback to get the robot info
    void robotInfoCallback ( const tuw_multi_robot_msgs::RobotInfo &_robotInfo );
    // Callback to get goal (single robot mode)
    void goalCallback ( const geometry_msgs::PoseStamped &_goal );
    // Callback to get a labelised goal (a goal for a single robot but in a multi-robot mode)
    void labelledGoalCallback ( const tuw_multi_robot_msgs::RobotGoals &_goal ); // R.Desarzens
    // Compute hash of a map
    size_t getHash ( const std::vector<signed char> &_map, const Eigen::Vector2d &_origin, const float &_resolution );
    // Compute hash of a graph
    size_t getHash ( const std::vector<Segment> &_graph );

    static bool sortSegments ( const Segment &i, const Segment &j )
    {
        return i.getSegmentId() < j.getSegmentId();
    }
    // TODO
    void unsubscribeTopic ( std::string _robot_name );
    // Helper function to compute the yaw
    float getYaw ( const geometry_msgs::Quaternion &_rot );
    // Extract radius from shape variable
    float calcRadius ( const int shape, const std::vector<float> &shape_variables ) const;
    // Prepare data to be used by the multi-robot router
    bool preparePlanning ( std::vector<float> &_radius, std::vector<Eigen::Vector3d> &_starts, std::vector<Eigen::Vector3d> &_goals, const tuw_multi_robot_msgs::RobotGoalsArray &_ros_goals, std::vector<std::string> &robot_names );
 
    // These members are for logging
    int attempts_total_;
    int attempts_successful_;
    double sum_processing_time_total_;
    double sum_processing_time_successful_;
    ros::Time time_first_robot_started_;

    tuw_multi_robot_msgs::RouterStatus mrrp_status_;

    dynamic_reconfigure::Server<tuw_multi_robot_router::routerConfig> param_server;
    dynamic_reconfigure::Server<tuw_multi_robot_router::routerConfig>::CallbackType call_type;
    ros::Publisher pubPlannerStatus_;

    ros::Subscriber subGoalSet_;
    ros::Subscriber subSingleRobotIdGoal_;              ///< subscriber to a labelised single goal
    ros::Subscriber subMap_;                            
    ros::Subscriber subSingleRobotGoal_;
    ros::Subscriber subVoronoiGraph_;
    ros::Subscriber subRobotInfo_;

    std::vector<RobotInfoPtr> subscribed_robots_;       ///< robots available
    std::vector<RobotInfoPtr> active_robots_;           ///< robots currently used by the planner
    std::map<std::string, double> finished_robots_;     ///< robots finished with execution time
    std::vector<std::string> missing_robots_;           ///< robots missing (TODO)
    float robot_radius_max_;
    cv::Mat distMap_;
    Eigen::Vector2d mapOrigin_;
    float mapResolution_;
  
    // Parameters 
    std::string frame_id_; 
    std::string singleRobotName_;
    bool single_robot_mode_;
    bool publish_routing_table_;
    float topic_timeout_s_ = 10;
    bool monitor_enabled_;
    
    // State variables 
    bool freshPlan_ = false;
    bool planner_prepared_ = false;
    bool plan_found_ = false;
    bool got_map_ = false;
    bool got_graph_ = false;
    tuw_multi_robot_msgs::RobotGoalsArray goals_msg_;   ///< goals received for each robot
    std::vector<Eigen::Vector3d> starts_;               ///< starting position of each robot
    std::vector<Eigen::Vector3d> goals_;                ///< goal of each robot
    std::vector<float> radius_;                         ///< radius of each robot
    std::vector<std::string> robot_names_;              ///< id of each robots
    std::vector<Segment> graph_;
    size_t current_map_hash_;
    size_t current_graph_hash_;
    int id_;

    // Robot registration - J. Mendes
    ros::ServiceServer register_service_;
    bool registerNewRobotCB(tuw_multi_robot_msgs::RegisterRobot::Request& req, tuw_multi_robot_msgs::RegisterRobot::Response& res);
    int num_of_robots_;
    int max_robots_;
    double noinfo_timeout_;
    std::vector<std::string> ids_;
    std::mutex reg_mutex_;

};
} // namespace multi_robot_router
#endif // Router_Node_H
