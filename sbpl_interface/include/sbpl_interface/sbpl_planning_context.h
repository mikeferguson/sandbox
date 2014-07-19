/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

#ifndef _MOVEIT_SBPL_INTERFACE_SBPL_PLANNING_CONTEXT_H_
#define _MOVEIT_SBPL_INTERFACE_SBPL_PLANNING_CONTEXT_H_

#include <moveit/robot_model/robot_model.h>
#include <sbpl_interface/environment_chain3d_moveit.h>  // for PlanningStatistics
#include <sbpl_interface/sbpl_planning_params.h>

namespace sbpl_interface
{

/**
 *  @brief Representation of a particular planning context -- the planning
 *         scene and the request are known, solution is not yet computed
 */
class SBPLPlanningContext : public planning_interface::PlanningContext
{
public:
  /// @sa planning_interface::PlanningContext::PlanningContext
  SBPLPlanningContext(const std::string& name, const std::string& group);

  virtual ~SBPLPlanningContext();

  bool init(const robot_model::RobotModelConstPtr& model,
            const sbpl_interface::SBPLPlanningParams& params);

  /// @sa planning_interface::PlanningContext::solve
  virtual bool solve(planning_interface::MotionPlanResponse& res);

  /// @sa planning_interface::PlanningContext::solve
  virtual bool solve(planning_interface::MotionPlanDetailedResponse& res);

  /// @sa planning_interface::PlanningContext::terminate
  virtual bool terminate();

  /// @sa planning_interface::PlanningContext::clear
  virtual void clear();

protected:
  ros::NodeHandle ph_;
  // TODO move this upstream so we don't thrash re-creating contexts
  ros::Publisher pub_;

  robot_model::RobotModelConstPtr robot_model_;
  sbpl_interface::SBPLPlanningParams sbpl_params_;

  PlanningStatistics last_planning_statistics_;
};

}  // namespace sbpl_interface

#endif  // _MOVEIT_SBPL_INTERFACE_SBPL_PLANNING_CONTEXT_H_
