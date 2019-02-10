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
#include <tf/tf.h>
#include <sstream> // std::stringstream
#include <iostream>

namespace gm = grid_map;

// COLORS DEFINITIONS
#define FG_BLACK "\33[0;30m"
#define FG_RED "\033[0;31m"
#define FG_GREEN "\033[0;32m"
#define FG_YELLOW "\033[0;33m"
#define FG_BLUE "\033[0;34m"
#define FG_MAGENTA "\033[0;35m"
#define FG_CYAN "\033[0;36m"
#define FG_L_RED "\033[0;91m"
#define FG_L_GREEN "\033[0;92m"
#define FG_L_YELLOW "\033[0;93m"
#define FG_L_BLUE "\033[0;94m"
#define FG_L_MAGENTA "\033[0;95m"
#define FG_L_CYAN "\033[0;96m"

#define FG_B_RED "\033[1;31m"
#define FG_B_GREEN "\033[1;32m"
#define FG_B_YELLOW "\033[1;33m"
#define FG_B_BLUE "\033[1;34m"
#define FG_B_MAGENTA "\033[1;35m"
#define FG_B_CYAN "\033[1;36m"
#define FG_B_L_RED "\033[1;91m"
#define FG_B_L_GREEN "\033[1;92m"
#define FG_B_L_YELLOW "\033[1;93m"
#define FG_B_L_BLUE "\033[1;94m"
#define FG_B_L_MAGENTA "\033[1;95m"
#define FG_B_L_CYAN "\033[1;96m"

#define BG_RED "\033[41m"
#define BG_GREEN "\033[42m"
#define BG_YELLOW "\033[43m"
#define BG_BLUE "\033[44m"
#define BG_MAGENTA "\033[45m"
#define BG_CYAN "\033[46m"
#define BG_L_RED "\033[101m"
#define BG_L_GREEN "\033[102m"
#define BG_L_YELLOW "\033[103m"
#define BG_L_BLUE "\033[104m"
#define BG_L_MAGENTA "\033[105m"
#define BG_L_CYAN "\033[106m"

#define RESET "\033[0m"

// TEST(GoalFinder, costmapUpdate)

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

// TODO more isGoalAttainable Asserts
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
    goal.pose.position.y = occupancyGrid.info.height*occupancyGrid.info.resolution +
                           occupancyGrid.info.origin.position.y + 0.01 + pointA.y;
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = 0;
    goal.pose.orientation.w = 1;

    ASSERT_EQ(true, gf.isGoalAttainable(goal));

    // Rotate of 90 deg and the footprint will be inside a lethal region
    yaw = M_PI_2;
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
    occupancyGrid.info.width = 300;
    occupancyGrid.info.height = 300;
    // occupancyGrid.info.origin.position.x = -occupancyGrid.info.resolution*occupancyGrid.info.width/2.;
    double center_x = std::rand()%201 - 100;
    double center_y = std::rand()%201 - 100;
    occupancyGrid.info.origin.position.x = center_x -occupancyGrid.info.resolution*occupancyGrid.info.width/2.;
    // occupancyGrid.info.origin.position.y = -occupancyGrid.info.resolution*occupancyGrid.info.height/2.;
    occupancyGrid.info.origin.position.y = center_y -occupancyGrid.info.resolution*occupancyGrid.info.height/2.;
    occupancyGrid.info.origin.orientation.w = 1.0;
    occupancyGrid.data.resize(occupancyGrid.info.width * occupancyGrid.info.height);

    for (auto& cell : occupancyGrid.data)
    {
        // cell = std::rand() % 102 - 1; // [-1, 100]
        cell = 0.; // [-1, 100]
    }
    occupancyGrid.data[occupancyGrid.info.width * (occupancyGrid.info.height/2-1) + occupancyGrid.info.width/2] = 100;
    // occupancyGrid.data[0] = 100;
    // occupancyGrid.data[occupancyGrid.info.width * occupancyGrid.info.height - 1] = 100;
    // std::cout << (occupancyGrid.info.width * occupancyGrid.info.height)/2 - 1 << std::endl;
    // gm::GridMap grid_from_og;
    // gm::GridMapRosConverter::fromOccupancyGrid(occupancyGrid, "local_costmap", grid_from_og);
    // printf("             Created map with size %f x %f m (%i x %i cells).\n"
    //          "The center of the map is located at (%f, %f) in the %s frame.\n",
    //          grid_from_og.getLength().x(), grid_from_og.getLength().y(),
    //          grid_from_og.getSize() (0), grid_from_og.getSize() (1),
    //          grid_from_og.getPosition().x(), grid_from_og.getPosition().y(), grid_from_og.getFrameId().c_str());

    // for (int i = 0; i < occupancyGrid.info.height; i++)
    // {
    //     for (int j = 0; j < occupancyGrid.info.width; j++)
    //     {
    //         double cost = grid_from_og.at("local_costmap", gm::Index(i, j));
    //         if (cost == 100)
    //         {
    //             std::cout << "Found! " << "x: " << i << " y: " << j << std::endl;
    //             gm::Position pos;
    //             grid_from_og.getPosition(gm::Index(i, j), pos);
    //             std::cout << "Found! " << "position: " << pos.x() << ", " << pos.y() << std::endl;
    //         }
    //     }
    // }

    // gm::GridMap grid({"local_costmap"});
    // grid.setFrameId("map");
    // grid.setGeometry(gm::Length(3.0, 3.0), 0.01, gm::Position(0.0, 0.0));
    // // grid["local_costmap"].setRandom();
    // grid["local_costmap"].setZero();
    // grid.atPosition("local_costmap", gm::Position(0.0, 0.0)) = 100.;
    // printf("             Created map with size %f x %f m (%i x %i cells).\n"
    //          "The center of the map is located at (%f, %f) in the %s frame.\n",
    //          grid.getLength().x(), grid.getLength().y(),
    //          grid.getSize() (0), grid.getSize() (1),
    //          grid.getPosition().x(), grid.getPosition().y(), grid.getFrameId().c_str());
    //
    // nav_msgs::OccupancyGrid occupancyGrid;
    // gm::GridMapRosConverter::toOccupancyGrid(grid, "local_costmap", -1.0, 100.0, occupancyGrid);

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
    // goal.pose.position.x = occupancyGrid.info.width*occupancyGrid.info.resolution/2.+occupancyGrid.info.origin.position.x;
    goal.pose.position.x = center_x;
    // goal.pose.position.y = occupancyGrid.info.height*occupancyGrid.info.resolution/2.+occupancyGrid.info.origin.position.y;
    goal.pose.position.y = center_y;
    goal.pose.position.z = 0;
    // random goal orientation (rotation around z)
    double yaw = (std::rand()/((RAND_MAX + 1u)/361)) / M_PI*180;
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = sin(yaw/2);
    goal.pose.orientation.w = cos(yaw/2);

    std::cout.setf(std::ios::fixed, std::ios::floatfield);

    std::cout << "             Original goal " << FG_CYAN << "(" <<
            std::setprecision(5) << goal.pose.position.x << ", " <<
            goal.pose.position.y << ", " <<
            tf::getYaw(goal.pose.orientation) << ")" << RESET << std::endl;

    ASSERT_EQ(false, gf.isGoalAttainable(goal));

    geometry_msgs::PoseStamped new_goal;
    bool found = gf.findNewGoal(goal, new_goal);

    std::cout << "             New goal      " << FG_CYAN << "(" <<
            std::setprecision(5) << new_goal.pose.position.x << ", " <<
            new_goal.pose.position.y << ", " <<
            tf::getYaw(new_goal.pose.orientation) << ")" << RESET << std::endl;

    geometry_msgs::Pose diff;

    // new goal "as seen from" original goal
    pose_cov_ops::inverseCompose(new_goal.pose, goal.pose, diff);

    std::cout << "             Inv_comp goal " << FG_B_L_CYAN << "(" <<
            std::setprecision(5) << diff.position.x << ", " <<
            diff.position.y << ", " <<
            tf::getYaw(diff.orientation) << ")" << RESET << std::endl;

    double dx = new_goal.pose.position.x - goal.pose.position.x;
    double dy = new_goal.pose.position.y - goal.pose.position.y;
    double dist = sqrt(dx*dx + dy*dy);
    double diff_norm = sqrt(diff.position.x*diff.position.x +
                            diff.position.y*diff.position.y);

    EXPECT_FLOAT_EQ(dist, diff_norm);

    if (gf.isGoalAttainable(goal) || !found)
    {
        // new goal should be equal to prev goal
        double pres_position = occupancyGrid.info.resolution;
        double pres_orientation = 1e-5;
        ASSERT_NEAR(0, diff.position.x, pres_position);
        ASSERT_NEAR(0, diff.position.x, pres_position);
        ASSERT_NEAR(0, diff.position.y, pres_position);
        ASSERT_NEAR(0, diff.orientation.x, pres_orientation);
        ASSERT_NEAR(0, diff.orientation.y, pres_orientation);
        ASSERT_NEAR(0, diff.orientation.z, pres_orientation);
        ASSERT_NEAR(1, diff.orientation.w, pres_orientation);
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

TEST(GoalFinder, findInRandomGrid)
{
    goal_finder::GoalFinder gf;
    std::srand(std::time(nullptr)); // use current time as seed for random generator

    // Create random occupancy grid
    nav_msgs::OccupancyGrid occupancyGrid;
    occupancyGrid.header.stamp = ros::Time(5.0);
    occupancyGrid.header.frame_id = "map";
    occupancyGrid.info.resolution = 0.01;
    occupancyGrid.info.width = 300;
    occupancyGrid.info.height = 300;
    double center_x = std::rand()%201 - 100;
    double center_y = std::rand()%201 - 100;
    occupancyGrid.info.origin.position.x = center_x -occupancyGrid.info.resolution*occupancyGrid.info.width/2.;
    occupancyGrid.info.origin.position.y = center_y -occupancyGrid.info.resolution*occupancyGrid.info.height/2.;
    occupancyGrid.info.origin.orientation.w = 1.0;
    occupancyGrid.data.resize(occupancyGrid.info.width * occupancyGrid.info.height);

    for (auto& cell : occupancyGrid.data)
    {
        cell = std::rand() % 102 - 1; // [-1, 100]
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
    goal.pose.position.x = center_x;
    goal.pose.position.y = center_y;
    goal.pose.position.z = 0;
    // random goal orientation (rotation around z)
    double yaw = (std::rand()/((RAND_MAX + 1u)/361)) / M_PI*180;
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = sin(yaw/2);
    goal.pose.orientation.w = cos(yaw/2);

    std::cout.setf(std::ios::fixed, std::ios::floatfield);

    std::cout << "             Original goal " << FG_CYAN << "(" <<
            std::setprecision(5) << goal.pose.position.x << ", " <<
            goal.pose.position.y << ", " <<
            tf::getYaw(goal.pose.orientation) << ")" << RESET << std::endl;

    geometry_msgs::PoseStamped new_goal;

        bool found = gf.findNewGoal(goal, new_goal);

    std::cout << "             New goal      " << FG_CYAN << "(" <<
            std::setprecision(5) << new_goal.pose.position.x << ", " <<
            new_goal.pose.position.y << ", " <<
            tf::getYaw(new_goal.pose.orientation) << ")" << RESET << std::endl;

    geometry_msgs::Pose diff;

    // new goal "as seen from" original goal
    pose_cov_ops::inverseCompose(new_goal.pose, goal.pose, diff);

    std::cout << "             Inv_comp goal " << FG_B_L_CYAN << "(" <<
            std::setprecision(5) << diff.position.x << ", " <<
            diff.position.y << ", " <<
            tf::getYaw(diff.orientation) << ")" << RESET << std::endl;

    double dx = new_goal.pose.position.x - goal.pose.position.x;
    double dy = new_goal.pose.position.y - goal.pose.position.y;
    double dist = sqrt(dx*dx + dy*dy);
    double diff_norm = sqrt(diff.position.x*diff.position.x +
                            diff.position.y*diff.position.y);

    EXPECT_FLOAT_EQ(dist, diff_norm);

    if (gf.isGoalAttainable(goal) || !found)
    {
        // new goal should be equal to prev goal
        double pres_position = occupancyGrid.info.resolution;
        double pres_orientation = 1e-5;
        ASSERT_NEAR(0, diff.position.x, pres_position);
        ASSERT_NEAR(0, diff.position.x, pres_position);
        ASSERT_NEAR(0, diff.position.y, pres_position);
        ASSERT_NEAR(0, diff.orientation.x, pres_orientation);
        ASSERT_NEAR(0, diff.orientation.y, pres_orientation);
        ASSERT_NEAR(0, diff.orientation.z, pres_orientation);
        ASSERT_NEAR(1, diff.orientation.w, pres_orientation);
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_finder_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
