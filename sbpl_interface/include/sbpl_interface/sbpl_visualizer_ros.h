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

#include <sbpl_interface/environment_chain3d_moveit.h>

namespace sbpl_interface
{

class SBPLVisualizerROS
{
public:

  SBPLVisualizerROS(ros::NodeHandle& nh)
  {
    bool publish;

    // TODO: goal states
    // TODO: distance field
    // TODO: heuristic

    nh.param("visual/expanded_states", publish, false);
    if (publish)
    {
      ROS_INFO("Create pub");
      expanded_states_pub_ = nh.advertise<visualization_msgs::Marker>("expanded_states", 1, true /* latched */);
    }

    // TODO: trajectory

    nh.param("visual/max_trajectory_points", max_trajectory_points_, 25);
  }

  /** @brief Publish the cells, in cartesian space, that were expanded */
  bool publishExpandedStates(EnvironmentChain3DMoveIt* env)
  {
    if (expanded_states_pub_)
    {
      visualization_msgs::Marker marker;
      env->getExpandedCellsVisualization(marker);
      expanded_states_pub_.publish(marker);
      return true;
    }
    return false;  // did not publish
  }


private:
  ros::Publisher goal_pub_;
  ros::Publisher distance_field_pub_;
  ros::Publisher hueristic_pub_;
  ros::Publisher expanded_states_pub_;
  ros::Publisher trajectory_pub_;

  int max_trajectory_points_;  /// maximum number of points in published trajectory
};

}  // namespace sbpl_interface

#endif  // _MOVEIT_SBPL_INTERFACE_SBPL_VISUALIZER_ROS_
