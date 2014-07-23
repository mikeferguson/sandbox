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
    cost_per_cell(1),
    cost_per_meter(50),
    use_bfs(true),
    angle_discretization(angles::from_degrees(1)),
    attempt_shortcut(true),
    interpolation_distance(angles::from_degrees(2)),
    planning_link_sphere_radius(0.1),
    field_resolution(0.02),
    field_x(2.0),
    field_y(2.0),
    field_z(2.0),
    field_origin_x(-0.5),
    field_origin_y(-1.0),
    field_origin_z(0.0),
    planner_params(60),
    use_joint_snap(true),
    joint_snap_thresh(0.1),
    use_xyzrpy_snap(true),
    xyzrpy_snap_thresh(0.2),
    use_rpy_snap(true)
  {
  }

  // Scoring
  int cost_per_cell;
  int cost_per_meter;

  // Environment
  bool use_bfs;  /// should we use BFS or Euclidean distance heuristic?
  double angle_discretization;  /// how is joint space chopped up in the discrete world?
  bool attempt_shortcut;  /// should we try to shortcut the path?
  double interpolation_distance;  /// for collision checking and shortcutting, how far apart can poses be?
  double planning_link_sphere_radius;  /// radius of sphere used to approximate point gripper for bfs heuristic

  // Environment - Distance Field
  double field_resolution;  /// resolution (m/cell) of field
  double field_x;           /// size of the distance field
  double field_y;
  double field_z;
  double field_origin_x;    /// origin of the distance field
  double field_origin_y;
  double field_origin_z;
  std::vector<std::string> field_links;  // links to add to distance field

  // Planner
  ReplanParams planner_params;

  // Motion Primitives
  std::vector<MotionPrimitivePtr> prims;
  bool use_joint_snap;
  double joint_snap_thresh;
  bool use_xyzrpy_snap;
  double xyzrpy_snap_thresh;
  bool use_rpy_snap;

  bool init(ros::NodeHandle& nh)
  {
    // Env
    nh.param("env/use_bfs_heuristic", use_bfs, use_bfs);
    nh.param("env/angle_discretization", angle_discretization, angle_discretization);
    nh.param("env/attempt_shortcut", attempt_shortcut, attempt_shortcut);
    nh.param("env/interpolation_distance", interpolation_distance, interpolation_distance);
    nh.param("env/planning_link_sphere_radius", planning_link_sphere_radius, planning_link_sphere_radius);

    // Distance Field
    nh.param("env/field/resolution", field_resolution, field_resolution);
    nh.param("env/field/size_x", field_x, field_x);
    nh.param("env/field/size_y", field_y, field_y);
    nh.param("env/field/size_z", field_z, field_z);
    nh.param("env/field/origin_x", field_origin_x, field_origin_x);
    nh.param("env/field/origin_y", field_origin_y, field_origin_y);
    nh.param("env/field/origin_z", field_origin_z, field_origin_z);

    // Robot Links To Insert Into Distance Field
    if (nh.hasParam("env/field/links"))
    {
      field_links.clear();
      nh.getParam("env/field/links", field_links);
    }

    // Planner
    nh.param("planner/initial_epsilon", planner_params.initial_eps, 5.0);
    nh.param("planner/final_epsilon", planner_params.final_eps, 1.0);
    nh.param("planner/decrement_epsilon", planner_params.dec_eps, 0.2);
    nh.param("planner/return_first_solution", planner_params.return_first_solution, false);
    nh.param("planner/max_time", planner_params.max_time, 60.0);
    nh.param("planner/repair_time", planner_params.repair_time, -1.0);

    // Adaptive Motion Primitives
    nh.param("prim/joint_snap", use_joint_snap, use_joint_snap);
    nh.param("prim/xyzrpy_snap", use_xyzrpy_snap, use_xyzrpy_snap);
    nh.param("prim/rpy_snap", use_rpy_snap, use_rpy_snap);

    nh.param("prim/joint_snap_thresh", joint_snap_thresh, joint_snap_thresh);
    nh.param("prim/xyzrpy_snap_thresh", xyzrpy_snap_thresh, xyzrpy_snap_thresh);

    // Simple Motion Primitives
    if (nh.hasParam("prim/simple"))
    {
      XmlRpc::XmlRpcValue prim_list;
      nh.getParam("prim/simple", prim_list);
      for (size_t i = 0; i < prim_list.size(); ++i)
      {
        ROS_ASSERT(prim_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(prim_list[i][0].getType() == XmlRpc::XmlRpcValue::TypeArray);
        std::vector<double> action;
        for (size_t j = 0; j < prim_list[i][0].size(); ++j)
          action.push_back(angles::from_degrees(static_cast<double>(prim_list[i][0][j])));

        MotionPrimitivePtr s;
        if (prim_list[i].size() == 3)
        {
          // motion primitive with min & max distance
          s.reset(new StaticMotionPrimitive(action,
                                            static_cast<double>(prim_list[i][1]),
                                            static_cast<double>(prim_list[i][2])));
        }
        else if (prim_list[i].size() == 2)
        {
          // motion primitive with min dist
          s.reset(new StaticMotionPrimitive(action,
                                            static_cast<double>(prim_list[i][1])));
        }
        else
        {
          s.reset(new StaticMotionPrimitive(action));
        }
        prims.push_back(s);
      }
    }
    else
    {
      // Load defaults
      for (int i = 0; i < 7; ++i)  /* TODO 7 needs to be parameterized */
      {
        std::vector<double> action(7,0.0);

        action[i] = angles::from_degrees(4);
        MotionPrimitivePtr s1(new StaticMotionPrimitive(action));
        prims.push_back(s1);

        action[i] = -action[i];
        MotionPrimitivePtr s2(new StaticMotionPrimitive(action));
        prims.push_back(s2);
      }
    }

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
    ROS_INFO_NAMED(stream,"%40s: %s", "Attempt Shortcut", attempt_shortcut ? "yes" : "no");
    ROS_INFO_NAMED(stream,"%40s: %0.3frad", "Interpolation distance", interpolation_distance);
    for (size_t i = 0; i < field_links.size(); ++i)
      ROS_INFO_NAMED(stream,"%40s: %s", "Distance field will contain", field_links[i].c_str());

    ROS_INFO_NAMED(stream,"Planner parameters:");
    ROS_INFO_NAMED(stream,"%40s: %0.3fm", "Initial epsilon", planner_params.initial_eps);
    ROS_INFO_NAMED(stream,"%40s: %0.3fm", "Final epsilon", planner_params.final_eps);
    ROS_INFO_NAMED(stream,"%40s: %0.3fm", "Epsilon decrement", planner_params.dec_eps);
    ROS_INFO_NAMED(stream,"%40s: %s", "Return after first solution ", planner_params.return_first_solution ? "yes (ignores time limits)" : "no");
    ROS_INFO_NAMED(stream,"%40s: %0.2fs", "Maximum planning time", planner_params.max_time);
    ROS_INFO_NAMED(stream,"%40s: %0.2fs", "Maximum planning time", planner_params.repair_time);
    ROS_INFO_NAMED(stream," ");

    // TODO: print motion primitives
  }

  void print()
  {
    print("sbpl");
  }
};

}  // namespace sbpl_interface

#endif  // _SBPL_INTERFACE_SBPL_PLANNING_PARAMS_H_
