/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Michael Ferguson
 *  Copyright (c) 2008, Maxim Likhachev
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Pennsylvania nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \Author Ben Cohen, Michael Ferguson */

#ifndef _MOVEIT_SBPL_INTERFACE_SBPL_VISUALIZER_ROS_
#define _MOVEIT_SBPL_INTERFACE_SBPL_VISUALIZER_ROS_

#include <algorithm>
#include <sbpl_interface/environment_chain3d_moveit.h>

namespace sbpl_interface
{

class SBPLVisualizerROS
{
public:

  SBPLVisualizerROS(ros::NodeHandle& nh) : num_states_(0)
  {
    nh.param("visual/goal", publish_goal_, false);
    nh.param("visual/field", publish_field_, false);
    nh.param("visual/expanded_states", publish_expanded_states_, false);
    nh.param("visual/trajectory", publish_trajectory_, false);
    nh.param("visual/max_trajectory_points", max_trajectory_points_, 25);

    publisher_ = nh.advertise<visualization_msgs::MarkerArray>("visualization", 10);
  }

  /** @brief Publish the cells, in cartesian space, that were expanded */
  bool publishExpandedStates(EnvironmentChain3DMoveIt* env)
  {
    if (!publish_expanded_states_)
      return false;  // did not publish

    if (!env)
      return false;

    std::vector< std::vector<double> > states;
    env->getExpandedStates(states);

    visualization_msgs::Marker marker;
    marker.header.frame_id = env->getPlanningFrame();
    marker.header.stamp = ros::Time::now();
    marker.ns = "expanded_states";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = env->getParams().field_resolution;
    marker.scale.y = env->getParams().field_resolution;
    marker.scale.z = env->getParams().field_resolution;
    marker.lifetime = ros::Duration(0);

    double max_cost = 1;  // avoid potential div/0
    for (size_t i = 0; i < states.size(); ++i)
    {
      if (states[i][3] > max_cost)
        max_cost = states[i][3];
    }

    visualization_msgs::MarkerArray array;
    array.markers.resize(std::max(states.size(), num_states_));
    for (size_t i = 0; i < states.size(); ++i)
    {
      array.markers[i] = marker;
      array.markers[i].id = i;
      array.markers[i].color.r = states[i][3]/max_cost;
      array.markers[i].color.g = (max_cost-states[i][3])/max_cost;
      array.markers[i].color.b = 0.0;
      array.markers[i].color.a = 1.0;
      array.markers[i].pose.position.x = states[i][0];
      array.markers[i].pose.position.y = states[i][1];
      array.markers[i].pose.position.z = states[i][2];
    }
    for (size_t i = states.size(); i < num_states_; ++i)
    {
      array.markers[i] = marker;
      array.markers[i].id = i;
      array.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    num_states_ = states.size();

    ROS_DEBUG_NAMED("sbpl", "Visualizing %d expanded states.", static_cast<int>(states.size()));
    publisher_.publish(array);
    return true;
  }

  /** @brief Publish the BFS */
  bool publishDistanceField(EnvironmentChain3DMoveIt* env)
  {
    if (!publish_field_)
      return false;  // did not publish

    if (!env)
      return false;

    distance_field::DistanceField* field = env->getDistanceField();

    visualization_msgs::MarkerArray array;
    array.markers.resize(1);
    field->getIsoSurfaceMarkers(0,  // min dist
                                0,  // max dist
                                env->getPlanningFrame(),
                                ros::Time::now(),
                                array.markers[0]);
    array.markers[0].color.r = 1.0;
    array.markers[0].color.g = 0.0;
    array.markers[0].color.b = 0.0;
    array.markers[0].color.a = 0.7;

    publisher_.publish(array);
    return true;
  }

private:
  ros::Publisher publisher_;

  size_t num_states_;  /// The number of states that were last published

  bool publish_goal_;
  bool publish_field_;
  bool publish_expanded_states_;
  bool publish_trajectory_;

  int max_trajectory_points_;  /// maximum number of points in published trajectory
};

}  // namespace sbpl_interface

#endif  // _MOVEIT_SBPL_INTERFACE_SBPL_VISUALIZER_ROS_
