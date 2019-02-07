#include "tuw_multi_robot_route_to_path/GoalFinder.h"
#include "tf/transform_listener.h"
#include "grid_map_ros/GridMapRosConverter.hpp"

// #include <costmap_2d/costmap_2d_ros.h>

namespace goal_finder {

GoalFinder::GoalFinder()
{
    ros::NodeHandle n("~/");
    tf::TransformListener tf(ros::Duration(10));

    cmap_sub_ = n.subscribe("/move_base/local_costmap/costmap", 1, &GoalFinder::costmapCallback, this);
    cmap_update_sub_ = n.subscribe("/move_base/local_costmap/costmap_updates", 1, &GoalFinder::costmapUpdateCallback, this);
    // cgoal_sub_ = n.subscribe("/move_base/current_goal", 1, &GoalFinder::currentGoalCallback, this);
}

// GoalFinder::currentGoalCallback(const geometry_msgs::PoseStamped& current_goal)
// {
//     ROS_INFO_STREAM("I heard a goal: " << current_goalt);
//     current_goal_ = current_goal;
// }

GoalFinder::~GoalFinder(){
}

void GoalFinder::costmapCallback(const nav_msgs::OccupancyGrid& occupancyGrid)
{
    ROS_INFO_STREAM("I heard a costmap w:" << occupancyGrid.info.width << ", h: " << occupancyGrid.info.height);

    // Convert to grid map.
    GridMapRosConverter::fromOccupancyGrid(occupancyGrid, "local_costmap", gridMap_);
}

void GoalFinder::costmapUpdateCallback(const map_msgs::OccupancyGridUpdate& update)
{
    ROS_INFO_STREAM("I heard a costmap update w:" << update.width << ", h: " << update.height);

    nav_msgs::OccupancyGrid occupancyGrid;
    GridMapRosConverter::toOccupancyGrid(gridMap_, "local_costmap", -1.0, 100.0, occupancyGrid);

    unsigned int x0 = update.x;
    unsigned int y0 = update.y;
    unsigned int xn = update.width  + x0;
    unsigned int yn = update.height + y0;

    unsigned int i = 0;
    for (unsigned int y = y0; y < yn; y++)
    {
        for (unsigned int x = x0; x < xn; x++)
        {
            occupancyGrid.data[update.width * y + x] = update.data[i++];
        }
    }
    GridMapRosConverter::fromOccupancyGrid(occupancyGrid, "local_costmap", gridMap_);

    // Use data from /goal_pose as position
    // transform to same frame_id as update.header.frame_id

    // gridMap_.atPosition("local_costmap", Position(goal.x, goal.y));
    // This will give the cost at the position (or a value meaning that the point is outside the map)
    // return if nothing worng
    // if something is wrong, iterate over gridMap_ and find a new free place
    // publish new goal pose
}

}
