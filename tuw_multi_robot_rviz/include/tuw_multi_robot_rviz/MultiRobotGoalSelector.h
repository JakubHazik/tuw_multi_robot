/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef MULTI_ROBOT_GOAL_SELECTOR_H
#define MULTI_ROBOT_GOAL_SELECTOR_H

#include <rviz/tool.h>
#include <ros/ros.h>

namespace Ogre
{
class SceneNode;
class Vector3;
}

namespace rviz
{
class VectorProperty;
class IntProperty;
class VisualizationManager;
class ViewportMouseEvent;
}

namespace tuw_multi_robot_rviz
{

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Tool.  Every tool
// which can be added to the tool bar is a subclass of
// rviz::Tool.
class MultiRobotGoalSelector: public rviz::Tool
{
Q_OBJECT
public:
  MultiRobotGoalSelector( );
  ~MultiRobotGoalSelector();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );
  

private:
  void makeFlag( const Ogre::Vector3& position );

  ros::NodeHandle nh_;
  ros::Publisher pubGoals_;
  std::vector<Ogre::SceneNode*> flag_nodes_;
  std::vector<rviz::VectorProperty*> vector_properties_;
  Ogre::SceneNode* moving_flag_node_;
  std::string flag_resource_;
  rviz::VectorProperty* current_flag_property_;
  rviz::IntProperty* nr_robtos_;
  
  int maxRobots_; 
  int robotCount_;
};
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials

#endif // PLANT_FLAG_TOOL_H