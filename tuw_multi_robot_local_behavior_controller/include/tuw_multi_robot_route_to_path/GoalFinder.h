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

#ifndef GOAL_FINDER_H
#define GOAL_FINDER_H

#include "grid_map_core/GridMap.hpp"
#include "geometry_msgs/PoseStamped.h"
// #include "grid_map_core/iterators/GridMapIterator.hpp"
// #include "grid_map_core/gtest_eigen.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include <tf/transform_listener.h>
#include <pose_cov_ops/pose_cov_ops.h>

// using namespace grid_map;
namespace gm = grid_map;

namespace goal_finder {

class GoalFinder
{

public:
    GoalFinder();
    ~GoalFinder();
    bool findNewGoal(double timeout, const geometry_msgs::PoseStamped&, geometry_msgs::PoseStamped&);
    bool isGoalAttainable(const geometry_msgs::PoseStamped&);

    bool transformFootprint(const geometry_msgs::PoseStamped&,
                            gm::Polygon&);

    void transformFootprint(const geometry_msgs::PoseStamped&,
                            const geometry_msgs::PolygonStamped&,
                            gm::Polygon&);

    inline static void transformFootprint(const std::string&,
                                   const tf::TransformListener&,
                                   const geometry_msgs::PoseStamped&,
                                   const geometry_msgs::PolygonStamped&,
                                   gm::Polygon&);

    void setCostmap(const nav_msgs::OccupancyGrid& cmap) { costmapCallback(cmap); } ;
    void setFootprint(const geometry_msgs::PolygonStamped& footprint) { footprintCallback(footprint); } ;

private:
    // Local costmap subscriptions
    ros::Subscriber cmap_sub_;
    ros::Subscriber cmap_update_sub_;
    ros::Subscriber footprint_sub_;

    std::string base_link_frame_id_;

    // Grid and grid frame
    std::string grid_frame_;
    gm::GridMap grid_map_;

    // TF
    tf::TransformListener tf_listener_;

    // geometry_msgs::PoseStamped current_goal_;

    // gm::Polygon footprint_;
    geometry_msgs::PolygonStamped footprint_;

    bool got_footprint_;
    bool got_costmap_;

    void costmapCallback(const nav_msgs::OccupancyGrid&);
    void costmapUpdateCallback(const map_msgs::OccupancyGridUpdate&);
    void footprintCallback(const geometry_msgs::PolygonStamped&);

};

typedef std::shared_ptr<goal_finder::GoalFinder> GoalFinderPtr;

void GoalFinder::transformFootprint(const std::string& base_link_frame_id,
                                    const tf::TransformListener& tf_listener,
                                    const geometry_msgs::PoseStamped& goal,
                                    const geometry_msgs::PolygonStamped& footprint,
                                    gm::Polygon& footprint_out)
{
    // Compute the polygon reprensenting the footprint of the robot at the
    // goal pose
    gm::Polygon gm_footprint;
    gm_footprint.setFrameId(goal.header.frame_id);

    // Verify that footprint polygon is expressed in the base_link frame
    std::string fp_frame = footprint.header.frame_id;
    if (fp_frame != base_link_frame_id)
    {
        if(tf_listener.waitForTransform(fp_frame,
                                        base_link_frame_id,
                                        ros::Time(0),
                                        ros::Duration(1.0)))
        {
            geometry_msgs::PointStamped point;
            point.header.frame_id = footprint.header.frame_id;
            for (const auto& vertex : footprint.polygon.points)
            {
                point.point.x = vertex.x;
                point.point.y = vertex.y;
                point.point.z = vertex.z;
                geometry_msgs::PointStamped point_at_base_link;
                geometry_msgs::Pose pose_at_goal;
                tf_listener.transformPoint(base_link_frame_id, point, point_at_base_link);
                geometry_msgs::Pose pose_at_base_link;
                pose_at_base_link.position = point_at_base_link.point;
                pose_at_base_link.orientation.x = 0;
                pose_at_base_link.orientation.y = 0;
                pose_at_base_link.orientation.z = 0;
                pose_at_base_link.orientation.w = 1;
                pose_cov_ops::compose(goal.pose, pose_at_base_link, pose_at_goal);
                gm_footprint.addVertex(gm::Position(pose_at_goal.position.x, pose_at_goal.position.y));
            }

        }
        else
        {
            ROS_ERROR("Goal Finder: transform between %s and %s is not available",
                      base_link_frame_id.c_str(), fp_frame.c_str());
            throw;
        }
    }
    else
    {
        geometry_msgs::Pose pose;
        for (const auto& vertex : footprint.polygon.points)
        {
            pose.position.x = vertex.x;
            pose.position.y = vertex.y;
            pose.position.z = vertex.z;
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = 0;
            pose.orientation.w = 1;
            geometry_msgs::Pose pose_at_goal;
            pose_cov_ops::compose(goal.pose, pose, pose_at_goal);
            // pose_cov_ops::compose(goal, point_at_base_link, point_at_goal);
            gm_footprint.addVertex(gm::Position(pose_at_goal.position.x,  pose_at_goal.position.y));
        }
    }
    footprint_out = gm_footprint;
};

};
#endif
