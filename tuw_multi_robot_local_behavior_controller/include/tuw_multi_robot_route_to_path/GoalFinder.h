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
* Author: José MENDES FILHO
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

using namespace grid_map;

namespace goal_finder {

class GoalFinder
{

public:
    GoalFinder();
    ~GoalFinder();

private:
    ros::Subscriber cmap_sub_;
    ros::Subscriber cmap_update_sub_;
    ros::Subscriber cgoal_sub_;
    GridMap gridMap_;
    // geometry_msgs::PoseStamped current_goal_;

    void costmapCallback(const nav_msgs::OccupancyGrid&);
    void costmapUpdateCallback(const map_msgs::OccupancyGridUpdate&);
};

typedef std::shared_ptr<goal_finder::GoalFinder> GoalFinderPtr;

};
#endif
