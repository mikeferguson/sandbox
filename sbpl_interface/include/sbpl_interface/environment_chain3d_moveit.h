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

#ifndef _ENVIRONMENT_CHAIN3D_MOVEIT_H_
#define _ENVIRONMENT_CHAIN3D_MOVEIT_H_

#include <sbpl_interface/environment_chain3d.h>
#include <sbpl_interface/bfs3d/BFS_3D.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>

namespace sbpl_interface
{

class EnvironmentChain3DMoveIt: public EnvironmentChain3D
{
public:
  EnvironmentChain3DMoveIt(const planning_scene::PlanningSceneConstPtr& planning_scene);
  virtual ~EnvironmentChain3DMoveIt();

  bool setupForMotionPlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                          const moveit_msgs::MotionPlanRequest &req,
                          moveit_msgs::MotionPlanResponse& res);
                          // TODO: should we add back PlanningParams?

  bool populateTrajectoryFromStateIDSequence(const std::vector<int>& state_ids,
                                             trajectory_msgs::JointTrajectory& traj) const;

protected:
  /** @brief Do collision checking, check path & joint limit constraints. */
  virtual bool isStateToStateValid(const std::vector<double>& start,
                                   const std::vector<double>& end);

  /** @brief Check goal constraints. */
  virtual bool isStateGoal(const std::vector<double>& angles);

  /** @brief Get the grid location of the end effector based on joint angles. */
  virtual bool getEndEffectorXYZ(const std::vector<double>& angles, int * xyz);

  /** @brief The heuristic based on BFS. */
  virtual int getEndEffectorHeuristic(int FromStateID, int ToStateID);

  planning_scene::PlanningSceneConstPtr planning_scene_;
  robot_state::RobotStatePtr state_;

  std::string planning_group_;
  const robot_model::JointModelGroup* joint_model_group_;
  const robot_model::LinkModel* tip_link_model_;

  Eigen::Affine3d goal_pose_;
  kinematic_constraints::KinematicConstraintSet goal_constraint_set_;
  kinematic_constraints::KinematicConstraintSet path_constraint_set_;

  BFS_3D *bfs_;
};

}  // namespace sbpl_interface

#endif  // _ENVIRONMENT_CHAIN3D_MOVEIT_H_