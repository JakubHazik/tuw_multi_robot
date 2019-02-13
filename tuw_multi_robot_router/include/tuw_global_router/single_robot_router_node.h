#ifndef SINGLE_ROBOT_ROUTER_NODE_H
#define SINGLE_ROBOT_ROUTER_NODE_H

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
#include <tf/transform_listener.h>

#include <tuw_global_router/router.h>
#include <tuw_global_router/mrr_utils.h>
#include <opencv/cv.hpp>
#include <mutex>
#include <tuw_multi_robot_msgs/RegisterRobot.h>
// #include <tuw_multi_robot_msgs/UnregisterRobot.h>

namespace multi_robot_router
{
class SingleRobotRouterNode : Router
{
public:
    /**
     * @brief Construct a new Router_Node object
     * @param n the nodehandle to register publishers and subscribers
     */
    SingleRobotRouterNode ( ros::NodeHandle &n );

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
    // Callback to get the map
    void mapCallback ( const nav_msgs::OccupancyGrid &_map );
    // Callback to get goal (single robot mode)
    void goalCallback ( const geometry_msgs::PoseStamped &_goal );
    // Callback to odometry (single robot mode)
    void odometryCallback( const nav_msgs::Odometry::ConstPtr & _odom);
    // Compute hash of a map
    size_t getHash ( const std::vector<signed char> &_map, const Eigen::Vector2d &_origin, const float &_resolution );
    // Compute hash of a graph
    size_t getHash ( const std::vector<Segment> &_graph );

    static bool sortSegments ( const Segment &i, const Segment &j )
    {
        return i.getSegmentId() < j.getSegmentId();
    }
    // TODO
    float getYaw ( const geometry_msgs::Quaternion &_rot );
    // Extract radius from shape variable
    float calcRadius ( const int shape, const std::vector<float> &shape_variables ) const;
 
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

    ros::Subscriber sub_map_;                            
    ros::Subscriber sub_robot_goal_;
    ros::Subscriber sub_voronoi_graph_;
    ros::Subscriber sub_odometry_;

    std::vector<RobotInfoPtr> active_robots_;           ///< robots currently used by the planner
    geometry_msgs::PoseStamped goal_;
    cv::Mat distMap_;
    Eigen::Vector2d mapOrigin_;
    float mapResolution_;

    tf::TransformListener tf_listener_;
  
    // Parameters 
    std::string frame_id_; 
    std::string robot_name_;
    double robot_radius_;
    bool publish_routing_table_;
    float topic_timeout_s_ = 10;
    bool monitor_enabled_;
    
    // State variables 
    bool freshPlan_ = false;
    bool plan_found_ = false;
    bool got_map_ = false;
    bool got_graph_ = false;
    std::vector<Segment> graph_;
    size_t current_map_hash_;
    size_t current_graph_hash_;
    int id_;

};
} // namespace multi_robot_router
#endif // Router_Node_H
