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

#include <gtest/gtest.h>
#include "ros/ros.h"
#include "tuw_multi_robot_route_to_path/GoalFinder.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PolygonStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include <pose_cov_ops/pose_cov_ops.h>

namespace gm = grid_map;

TEST(GoalFinder, isGoalAttainable)
{
    goal_finder::GoalFinder gf;
    std::srand(std::time(nullptr)); // use current time as seed for random generator

    // Create goal
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    // goal position will be well outside the grid map, at first
    goal.pose.position.x = 1000;
    goal.pose.position.y = 1000;
    goal.pose.position.z = 0;
    // random goal orientation (rotation around z)
    double yaw = (std::rand()/((RAND_MAX + 1u)/361)) /    M_PI*180;
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = sin(yaw/2);
    goal.pose.orientation.w = cos(yaw/2);

    // If no footprint nor costmap are set Goal Finder should consider the goal attainable
    ASSERT_EQ(true, gf.isGoalAttainable(goal));

    // Create occupancy grid
    nav_msgs::OccupancyGrid occupancyGrid;
    occupancyGrid.header.stamp = ros::Time(5.0);
    occupancyGrid.header.frame_id = "map";
    occupancyGrid.info.resolution = 0.01;
    occupancyGrid.info.width = 200;
    occupancyGrid.info.height = 200;
    occupancyGrid.info.origin.position.x = 0.0;
    occupancyGrid.info.origin.position.y = 0.0;
    occupancyGrid.info.origin.orientation.w = 1.0;
    occupancyGrid.data.resize(occupancyGrid.info.width * occupancyGrid.info.height);

    // Costmap of only lethal costs
    for (auto& cell : occupancyGrid.data)
    {
        // cell = std::rand() % 102 - 1; // [-1, 100]
        cell = 100.; // [-1, 100]
    }

    // Create footprint (rectangle 1.6 x 0.4)
    geometry_msgs::PolygonStamped footprint;
    footprint.header.frame_id = "base_link";
    geometry_msgs::Point32 pointA, pointB, pointC, pointD;
    pointA.x =  0.8;
    pointA.y =  0.2;
    pointA.z =  0.0;
    footprint.polygon.points.push_back(pointA);
    pointB.x =  0.8;
    pointB.y = -0.2;
    pointB.z =  0.0;
    footprint.polygon.points.push_back(pointB);
    pointC.x = -0.8;
    pointC.y = -0.2;
    pointC.z =  0.0;
    footprint.polygon.points.push_back(pointC);
    pointD.x = -0.8;
    pointD.y =  0.2;
    pointD.z =  0.0;
    footprint.polygon.points.push_back(pointD);

    gf.setCostmap(occupancyGrid);
    gf.setFootprint(footprint);

    // Goal is outside the costmap, should be attainable
    ASSERT_EQ(true, gf.isGoalAttainable(goal));

    // Goal position in the middle of the costmap
    goal.pose.position.x = occupancyGrid.info.width*occupancyGrid.info.resolution/2.+occupancyGrid.info.origin.position.x;
    goal.pose.position.y = occupancyGrid.info.height*occupancyGrid.info.resolution/2.+occupancyGrid.info.origin.position.y;

    // There will be 100s inside the footprint at goal
    ASSERT_EQ(false, gf.isGoalAttainable(goal));

    // Goal position just outside the costmap and given the dimension and orientation should be attainable
    goal.pose.position.x = occupancyGrid.info.width*occupancyGrid.info.resolution/2.+occupancyGrid.info.origin.position.x;
    goal.pose.position.y = occupancyGrid.info.height*occupancyGrid.info.resolution+occupancyGrid.info.origin.position.y + 0.01 + pointA.y;
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = 0;
    goal.pose.orientation.w = 1;

    ASSERT_EQ(true, gf.isGoalAttainable(goal));

    // Rotate of 90deg and the footprint will be inside a lethal region
    yaw = M_PI/2;
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = sin(yaw/2);
    goal.pose.orientation.w = cos(yaw/2);

    ASSERT_EQ(false, gf.isGoalAttainable(goal));

    // A free space costmap
    for (auto& cell : occupancyGrid.data)
    {
        cell = 0.; // [-1, 100]
    }
    gf.setCostmap(occupancyGrid);

    // Undependent from the goal pose, it should be attainable
    ASSERT_EQ(true, gf.isGoalAttainable(goal));

    // Goal position in the middle of the costmap
    goal.pose.position.x = occupancyGrid.info.width*occupancyGrid.info.resolution/2.+occupancyGrid.info.origin.position.x;
    goal.pose.position.y = occupancyGrid.info.height*occupancyGrid.info.resolution/2.+occupancyGrid.info.origin.position.y;

    // There will be no 100s inside the footprint at goal
    ASSERT_EQ(true, gf.isGoalAttainable(goal));

}

TEST(GoalFinder, findNewGoal)
{
    goal_finder::GoalFinder gf;
    std::srand(std::time(nullptr)); // use current time as seed for random generator

    // Create random occupancy grid
    nav_msgs::OccupancyGrid occupancyGrid;
    occupancyGrid.header.stamp = ros::Time(5.0);
    occupancyGrid.header.frame_id = "map";
    occupancyGrid.info.resolution = 0.01;
    occupancyGrid.info.width = 200;
    occupancyGrid.info.height = 200;
    occupancyGrid.info.origin.position.x = 0.0;
    occupancyGrid.info.origin.position.y = 0.0;
    occupancyGrid.info.origin.orientation.w = 1.0;
    occupancyGrid.data.resize(occupancyGrid.info.width * occupancyGrid.info.height);

    for (auto& cell : occupancyGrid.data)
    {
        cell = std::rand() % 102 - 1; // [-1, 100]
        // cell = 100.; // [-1, 100]
    }

    // Create footprint (rectangle 1.6 x 0.4)
    geometry_msgs::PolygonStamped footprint;
    footprint.header.frame_id = "base_link";
    geometry_msgs::Point32 pointA, pointB, pointC, pointD;
    pointA.x =  0.8;
    pointA.y =  0.2;
    pointA.z =  0.0;
    footprint.polygon.points.push_back(pointA);
    pointB.x =  0.8;
    pointB.y = -0.2;
    pointB.z =  0.0;
    footprint.polygon.points.push_back(pointB);
    pointC.x = -0.8;
    pointC.y = -0.2;
    pointC.z =  0.0;
    footprint.polygon.points.push_back(pointC);
    pointD.x = -0.8;
    pointD.y =  0.2;
    pointD.z =  0.0;
    footprint.polygon.points.push_back(pointD);

    gf.setCostmap(occupancyGrid);
    gf.setFootprint(footprint);

    // Create goal
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    // goal position in the middle of the costmap
    goal.pose.position.x = occupancyGrid.info.width*occupancyGrid.info.resolution/2.+occupancyGrid.info.origin.position.x;
    goal.pose.position.y = occupancyGrid.info.height*occupancyGrid.info.resolution/2.+occupancyGrid.info.origin.position.y;
    goal.pose.position.z = 0;
    // random goal orientation (rotation around z)
    double yaw = (std::rand()/((RAND_MAX + 1u)/361)) / M_PI*180;
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = sin(yaw/2);
    goal.pose.orientation.w = cos(yaw/2);

    geometry_msgs::PoseStamped new_goal;
    bool found = gf.findNewGoal(goal, new_goal);

    geometry_msgs::Pose diff;
    pose_cov_ops::inverseCompose(goal.pose, new_goal.pose, diff);

    if (gf.isGoalAttainable(goal) || !found)
    {
        // new goal should be equal to prev goal

        ASSERT_EQ(0, diff.position.x);
        ASSERT_EQ(0, diff.position.y);
        ASSERT_EQ(0, diff.orientation.x);
        ASSERT_EQ(0, diff.orientation.y);
        ASSERT_EQ(0, diff.orientation.z);
        ASSERT_EQ(1, diff.orientation.w);
    }
    else
    {
        // new goal should be different from prev goal

        bool diff_not_zero = diff.position.x != 0. ||
                             diff.position.y != 0. ||
                             diff.orientation.x != 0. ||
                             diff.orientation.y != 0. ||
                             diff.orientation.z != 0. ||
                             diff.orientation.w != 1.;
        ASSERT_EQ(true, diff_not_zero);
    }
}

TEST(GoalFinder, transformFootprint)
{
    // goal_finder::GoalFinder * gf = new goal_finder::GoalFinder();
    goal_finder::GoalFinder gf;

    std::srand(std::time(nullptr)); // use current time as seed for random generator

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "base_link";
    // random goal position
    goal.pose.position.x = std::rand();
    goal.pose.position.y = std::rand();
    goal.pose.position.z = std::rand();
    // random goal orientation (rotation around z)
    const double yaw = (std::rand()/((RAND_MAX + 1u)/361)) /    M_PI*180;
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = sin(yaw/2);
    goal.pose.orientation.w = cos(yaw/2);

    // 4 random points footprint
    geometry_msgs::PolygonStamped footprint;
    footprint.header.frame_id = "base_link";
    geometry_msgs::Point32 pointA, pointB, pointC, pointD;
    pointA.x = std::rand();
    pointA.y = std::rand();
    pointA.z = std::rand();
    footprint.polygon.points.push_back(pointA);
    pointB.x = std::rand();
    pointB.y = std::rand();
    pointB.z = std::rand();
    footprint.polygon.points.push_back(pointB);
    pointC.x = std::rand();
    pointC.y = std::rand();
    pointC.z = std::rand();
    footprint.polygon.points.push_back(pointC);
    pointD.x = std::rand();
    pointD.y = std::rand();
    pointD.z = std::rand();
    footprint.polygon.points.push_back(pointD);

    gm::Polygon gm_footprint;

    gf.transformFootprint(goal, footprint, gm_footprint);
    const double pres = 1e-5; // less than 1e-5 may fail
    ASSERT_NEAR(pointA.x*cos(yaw)-pointA.y*sin(yaw)+goal.pose.position.x, gm_footprint.getVertices().at(0).x(), pres);
    ASSERT_NEAR(pointA.x*sin(yaw)+pointA.y*cos(yaw)+goal.pose.position.y, gm_footprint.getVertices().at(0).y(), pres);
    ASSERT_NEAR(pointB.x*cos(yaw)-pointB.y*sin(yaw)+goal.pose.position.x, gm_footprint.getVertices().at(1).x(), pres);
    ASSERT_NEAR(pointB.x*sin(yaw)+pointB.y*cos(yaw)+goal.pose.position.y, gm_footprint.getVertices().at(1).y(), pres);
    ASSERT_NEAR(pointC.x*cos(yaw)-pointC.y*sin(yaw)+goal.pose.position.x, gm_footprint.getVertices().at(2).x(), pres);
    ASSERT_NEAR(pointC.x*sin(yaw)+pointC.y*cos(yaw)+goal.pose.position.y, gm_footprint.getVertices().at(2).y(), pres);
    ASSERT_NEAR(pointD.x*cos(yaw)-pointD.y*sin(yaw)+goal.pose.position.x, gm_footprint.getVertices().at(3).x(), pres);
    ASSERT_NEAR(pointD.x*sin(yaw)+pointD.y*cos(yaw)+goal.pose.position.y, gm_footprint.getVertices().at(3).y(), pres);
    // for (const auto& vertex : gm_footprint.getVertices()){}
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_finder_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
