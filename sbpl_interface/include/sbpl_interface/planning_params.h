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

#ifndef _SBPL_INTERFACE_PLANNING_PARAMS_H_
#define _SBPL_INTERFACE_PLANNING_PARAMS_H_

#include <angles/angles.h>
#include <sbpl_interface/motion_primitives.h>

namespace sbpl_interface
{

struct PlanningParams
{
  PlanningParams() :
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
    planning_link_sphere_radius(0.05)
  {
    // TODO: this is a huge hack, load prims from a file or something
    // Setup default mprims
    for (int i = 0; i < 7; ++i)
    {
      std::vector<double> action(7,0.0);

      action[i] = angles::from_degrees(4);
      StaticMotionPrimitive s1(action);
      prims.push_back(s1);

      action[i] = -action[i];
      StaticMotionPrimitive s2(action);
      prims.push_back(s2);    
    }
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

  // Motion Primitives
  std::vector<MotionPrimitive> prims;
};

}  // namespace sbpl_interface

#endif  // _SBPL_INTERFACE_PLANNING_PARAMS_H_
