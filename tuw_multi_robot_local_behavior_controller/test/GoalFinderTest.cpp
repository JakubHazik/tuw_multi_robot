#include "ros/ros.h"
#include "tuw_multi_robot_route_to_path/GoalFinder.h"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PolygonStamped.h"
// #include "nav_msgs/OccupancyGrid.h"
// #include "map_msgs/OccupancyGridUpdate.h"
// #include <tf/transform_listener.h>

// using namespace grid_map;
namespace gm = grid_map;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_finder");
    goal_finder::GoalFinder * gf = new goal_finder::GoalFinder();

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "base_link";
    // 1 unit shift in x
    goal.pose.position.x = 1;
    goal.pose.position.y = 0;
    goal.pose.position.z = 0;
    // +45 deg rotaion
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = 0.3826834;
    goal.pose.orientation.w = 0.9238795;

    // Centered square footprint of size 2x2
    geometry_msgs::PolygonStamped footprint;
    footprint.header.frame_id = "base_link";
    geometry_msgs::Point32 point;
    point.x = 1;
    point.y = 1;
    point.z = 0.0;
    footprint.polygon.points.push_back(point);
    point.x = 1;
    point.y = -1;
    point.z = 0.0;
    footprint.polygon.points.push_back(point);
    point.x = -1;
    point.y = -1;
    point.z = 0.0;
    footprint.polygon.points.push_back(point);
    point.x = -1;
    point.y = 1;
    point.z = 0.0;
    footprint.polygon.points.push_back(point);

    gm::Polygon gm_footprint;

    gf->transformFootprint(goal, footprint, gm_footprint);
    for (const auto& vertex : gm_footprint.getVertices())
    {
        std::cout << vertex.x() << ", " << vertex.y() << std::endl;
    }
    // Expected output
    // 1, 1.41421
    // 2.41421, -1.34228e-07
    // 1, -1.41421
    // -0.414214, 1.34228e-07

    ros::spin();
    delete gf;
    return 0;
}
