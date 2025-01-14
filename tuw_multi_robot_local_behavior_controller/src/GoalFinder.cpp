/*******************************************************************************
* 2-Clause BSD License
*
* Copyright (c) 2019, iFollow Robotics
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Author: José MENDES FILHO (mendesfilho@pm.me)
*******************************************************************************/

#include "tuw_multi_robot_route_to_path/GoalFinder.h"
#include "tuw_multi_robot_route_to_path/GoalFinderOpt.h"
#include "tf/transform_listener.h"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "tuw_multi_robot_route_to_path/GoalFinderOpt.h"

namespace goal_finder {

GoalFinder::GoalFinder()
    : grid_frame_("")
    , got_footprint_(false)
    , got_costmap_(false)
{
    ros::NodeHandle n("~/");
    tf::TransformListener tf(ros::Duration(10));

    // TODO
    // stop using this hardcoded topic names. Get params (or at least do remaps)
    base_link_frame_id_ = "base_link";
    cmap_sub_ = n.subscribe("move_base/local_costmap/costmap", 1, &GoalFinder::costmapCallback, this);
    cmap_update_sub_ = n.subscribe("move_base/local_costmap/costmap_updates", 1, &GoalFinder::costmapUpdateCallback, this);
    footprint_sub_ = n.subscribe("move_base/local_costmap/obstacle_layer_footprint/footprint_stamped", 1, &GoalFinder::footprintCallback, this);
    // cgoal_sub_ = n.subscribe("/move_base/current_goal", 1, &GoalFinder::currentGoalCallback, this);
}

// GoalFinder::currentGoalCallback(const geometry_msgs::PoseStamped& current_goal)
// {
//     ROS_INFO_STREAM("I heard a goal: " << current_goalt);
//     current_goal_ = current_goal;
// }

GoalFinder::~GoalFinder()
{
}

void GoalFinder::transformFootprint(const geometry_msgs::PoseStamped& goal,
                                    const geometry_msgs::PolygonStamped& footprint,
                                    gm::Polygon& footprint_out)
{
    GoalFinder::transformFootprint(base_link_frame_id_,
                                   tf_listener_,
                                   goal,
                                   footprint,
                                   footprint_out);
}

bool GoalFinder::transformFootprint(const geometry_msgs::PoseStamped& goal,
                                    gm::Polygon& footprint_out)
{
    if (!got_footprint_)
    {
        return false;
    }
    transformFootprint(goal, footprint_, footprint_out);
    return true;
}

bool GoalFinder::isGoalAttainable(const geometry_msgs::PoseStamped& last_goal_sent)
{
    if (!got_footprint_ || !got_costmap_)
    {
        return true;
    }

    geometry_msgs::PoseStamped goal_cmap_frame = last_goal_sent;
    if (last_goal_sent.header.frame_id != grid_frame_)
    {
        if (tf_listener_.waitForTransform(last_goal_sent.header.frame_id,
                                          grid_frame_,
                                          ros::Time(0),
                                          ros::Duration(1.0)))
        {
            tf_listener_.transformPose(grid_frame_, last_goal_sent, goal_cmap_frame);
        }
        else
        {
            ROS_ERROR("Goal Finder: transform between %s and %s is not available",
                      grid_frame_.c_str(), last_goal_sent.header.frame_id.c_str());
            throw;
        }
    }

    gm::Polygon gm_footprint;
    transformFootprint(goal_cmap_frame, gm_footprint);

    bool attainable = true;
    for (gm::PolygonIterator iterator(grid_map_, gm_footprint);
         !iterator.isPastEnd(); ++iterator)
    {
        if (grid_map_.at("local_costmap", *iterator) == 100)
        {
            // Have to find new goal pose
            attainable = false;
            break;
        }
    }
    return attainable;
}

bool GoalFinder::findNewGoal(double timeout,
                             const geometry_msgs::PoseStamped& last_goal_sent,
                             geometry_msgs::PoseStamped& new_goal)
{
    if (!got_footprint_ || !got_costmap_)
    {
        new_goal = last_goal_sent;
        return true;
    }

    geometry_msgs::PoseStamped goal_cmap_frame = last_goal_sent;
    if (last_goal_sent.header.frame_id != grid_frame_)
    {
        if (tf_listener_.waitForTransform(last_goal_sent.header.frame_id,
                                          grid_frame_,
                                          ros::Time(0),
                                          ros::Duration(1.0)))
        {
            tf_listener_.transformPose(grid_frame_, last_goal_sent, goal_cmap_frame);
        }
        else
        {
            ROS_ERROR("Goal Finder: transform between %s and %s is not available",
                      grid_frame_.c_str(), last_goal_sent.header.frame_id.c_str());
            throw;
        }
    }

    gm::Polygon gm_footprint;
    transformFootprint(goal_cmap_frame, gm_footprint);

    bool find_new_flag = false;
    for (gm::PolygonIterator iterator(grid_map_, gm_footprint);
         !iterator.isPastEnd(); ++iterator)
    {
        if (grid_map_.at("local_costmap", *iterator) == 100)
        {
            // Have to find new goal pose
            find_new_flag = true;
            break;
        }
    }

    if (find_new_flag)
    {
        if (goal_finder_opt::findOptGoal(ceil(timeout), base_link_frame_id_, tf_listener_,
                                         grid_map_, footprint_,
                                         goal_cmap_frame, new_goal))
        {
            ROS_DEBUG("Goal Finder: a new goal was found");
            return true;
        }
        ROS_DEBUG("Goal Finder: failed to find a new goal");
        return false;

    }
    else
    {
        ROS_DEBUG("Goal Finder: the current goal is actually fine");
        new_goal = last_goal_sent;
        return true;
    }
}

void GoalFinder::footprintCallback(const geometry_msgs::PolygonStamped& footprint)
{
    ROS_DEBUG("Goal Finder: footprint callback");
    footprint_ = footprint;
    got_footprint_ = true;
}

void GoalFinder::costmapCallback(const nav_msgs::OccupancyGrid& occupancy_grid)
{
    ROS_DEBUG_STREAM("Goal Finder: costmap callback, w:" << occupancy_grid.info.width << ", h: " << occupancy_grid.info.height);

    // Convert to grid map.
    gm::GridMapRosConverter::fromOccupancyGrid(occupancy_grid, "local_costmap", grid_map_);
    grid_frame_ = occupancy_grid.header.frame_id;
    got_costmap_ = true;
}

void GoalFinder::costmapUpdateCallback(const map_msgs::OccupancyGridUpdate& update)
{
    ROS_DEBUG_STREAM("Goal Finder: costmap update callback, w:" << update.width << ", h: " << update.height);

    if (got_costmap_ == false)
    {
        ROS_ERROR("Costmap update received before costmap");
        throw "Costmap update received before costmap";
    }
    else if (update.header.frame_id != grid_frame_)
    {
        ROS_ERROR("Costmap update frame_id is different from costmap frame_id");
        throw "Costmap update frame_id is different from costmap frame_id";
    }

    nav_msgs::OccupancyGrid occupancy_grid;
    gm::GridMapRosConverter::toOccupancyGrid(grid_map_, "local_costmap", -1.0, 100.0, occupancy_grid);

    unsigned int x0 = update.x;
    unsigned int y0 = update.y;
    unsigned int xn = update.width  + x0;
    unsigned int yn = update.height + y0;

    unsigned int i = 0;
    for (auto y = y0; y < yn; y++)
    {
        for (auto x = x0; x < xn; x++)
        {
            occupancy_grid.data[update.width * y + x] = update.data[i++];
        }
    }
    gm::GridMapRosConverter::fromOccupancyGrid(occupancy_grid, "local_costmap", grid_map_);
}

}
