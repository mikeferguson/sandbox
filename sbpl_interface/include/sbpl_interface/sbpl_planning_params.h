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

#ifndef _SBPL_INTERFACE_SBPL_PLANNING_PARAMS_H_
#define _SBPL_INTERFACE_SBPL_PLANNING_PARAMS_H_

#include <ros/ros.h>
#include <angles/angles.h>
#include <sbpl/planners/planner.h>
#include <sbpl_interface/motion_primitives.h>

namespace sbpl_interface
{

struct SBPLPlanningParams
{
  SBPLPlanningParams() :
    use_bfs(true),
    cost_per_cell(1),
    cost_per_meter(50),
    field_resolution(0.02),
    field_x(2.0),
    field_y(2.0),
    field_z(2.0),
    field_origin_x(-0.5),
    field_origin_y(-1.0),
    field_origin_z(0.0),
    planning_link_sphere_radius(0.05),
    angle_discretization(angles::from_degrees(1)),
    planner_params(60)
  {
  }

  // Algorithms
  bool use_bfs;             /// should we use BFS or Euclidean distance heuristic?

  // Scoring
  int cost_per_cell;
  int cost_per_meter;

  // Distance Field
  double field_resolution;  /// resolution (m/cell) of field
  double field_x;           /// size of the distance field
  double field_y;
  double field_z;
  double field_origin_x;    /// origin of the distance field
  double field_origin_y;
  double field_origin_z;

  // Other
  double planning_link_sphere_radius;  /// radius of sphere used to approximate point gripper for bfs heuristic
  double angle_discretization;  /// How is joint space chopped up in the discrete world?

  // Planner
  ReplanParams planner_params;

  // Motion Primitives
  std::vector<MotionPrimitivePtr> prims;

  bool init(ros::NodeHandle& nh)
  {
    // Env
    nh.param("env/use_bfs", use_bfs, true);

    // Motion Primitives
    for (int i = 0; i < 7; ++i)
    {
      // TODO: replace this hack with a real loader of parameters
      std::vector<double> action(7,0.0);

      action[i] = angles::from_degrees(4);
      MotionPrimitivePtr s1(new StaticMotionPrimitive(action));
      prims.push_back(s1);

      action[i] = -action[i];
      MotionPrimitivePtr s2(new StaticMotionPrimitive(action));
      prims.push_back(s2);
    }

    // TODO: add complex MPs

    // Planner
    nh.param("planner/initial_epsilon", planner_params.initial_eps, 5.0);
    nh.param("planner/final_epsilon", planner_params.final_eps, 1.0);
    nh.param("planner/decrement_epsilon", planner_params.dec_eps, 0.2);
    nh.param("planner/return_first_solution", planner_params.return_first_solution, false);
    nh.param("planner/max_time", planner_params.max_time, 60.0);
    nh.param("planner/repair_time", planner_params.repair_time, -1.0);

    return true;
  }

  bool init()
  {
    ros::NodeHandle nh("~");
    init(nh);
  }

  void print(std::string stream)
  {
    ROS_INFO_NAMED(stream," ");
    ROS_INFO_NAMED(stream,"Environment parameters:");
    ROS_INFO_NAMED(stream,"%40s: %s", "Use BFS", use_bfs ? "yes" : "no");
    ROS_INFO_NAMED(stream,"Planner parameters:");
    ROS_INFO_NAMED(stream,"%40s: %0.3fm", "Initial epsilon", planner_params.initial_eps);
    ROS_INFO_NAMED(stream,"%40s: %0.3fm", "Final epsilon", planner_params.final_eps);
    ROS_INFO_NAMED(stream,"%40s: %0.3fm", "Epsilon decrement", planner_params.dec_eps);
    ROS_INFO_NAMED(stream,"%40s: %s", "Return after first solution ", planner_params.return_first_solution ? "yes (ignores time limits)" : "no");
    ROS_INFO_NAMED(stream,"%40s: %0.2fs", "Maximum planning time", planner_params.max_time);
    ROS_INFO_NAMED(stream,"%40s: %0.2fs", "Maximum planning time", planner_params.repair_time);
    ROS_INFO_NAMED(stream," ");
  }

  void print()
  {
    print("sbpl");
  }
};

}  // namespace sbpl_interface

#endif  // _SBPL_INTERFACE_SBPL_PLANNING_PARAMS_H_
