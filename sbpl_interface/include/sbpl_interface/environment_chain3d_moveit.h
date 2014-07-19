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

#include <sbpl_interface/bfs3d/BFS_3D.h>
#include <sbpl_interface/environment_chain3d.h>
#include <moveit/distance_field/propagation_distance_field.h>
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
  EnvironmentChain3DMoveIt();
  virtual ~EnvironmentChain3DMoveIt();

  bool setupForMotionPlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                          const moveit_msgs::MotionPlanRequest &req,
                          moveit_msgs::MotionPlanResponse& res,
                          SBPLPlanningParams& params);

  bool populateTrajectoryFromStateIDSequence(const std::vector<int>& state_ids,
                                             trajectory_msgs::JointTrajectory& traj) const;

protected:
  /** @brief Do collision checking, check path & joint limit constraints. */
  virtual bool isStateToStateValid(const std::vector<double>& start,
                                   const std::vector<double>& end);

  /** @brief Check goal constraints. */
  virtual bool isStateGoal(const std::vector<double>& angles);

  /** @brief Get the discrete XYZ of the end effector based on joint angles. */
  virtual bool getEndEffectorCoord(const std::vector<double>& angles, int * xyz);
  virtual bool continuousXYZtoDiscreteXYZ(const double X, const double Y, const double Z,
                                          int& x, int& y, int& z);

  /** @brief The heuristic based on BFS. */
  virtual int getEndEffectorHeuristic(int x, int y, int z);

  planning_scene::PlanningSceneConstPtr planning_scene_;
  robot_state::RobotStatePtr state_;

  std::string planning_group_;
  const robot_model::JointModelGroup* joint_model_group_;
  const robot_model::LinkModel* tip_link_model_;

  Eigen::Affine3d goal_pose_;
  kinematic_constraints::KinematicConstraintSet* goal_constraint_set_;
  kinematic_constraints::KinematicConstraintSet* path_constraint_set_;

  distance_field::DistanceField* field_;
  BFS_3D *bfs_;
};

/**
 *  \brief Fill in the BFS walls from a DistanceField
 *  \returns The number of walls filled in.
 */
inline int fillBFSfromField(distance_field::DistanceField* field,
                            BFS_3D * bfs,
                            SBPLPlanningParams& params)
{
  int walls = 0;
  for (int z = 1; z < field->getZNumCells() - 1; ++z)
    for (int y = 1; y < field->getYNumCells() - 1; ++y)
      for (int x = 1; x < field->getXNumCells() - 1; ++x)
        if (field->getDistance(x,y,z) <= params.planning_link_sphere_radius)
        {
          bfs->setWall(x, y, z);
          walls++;
        }
  return walls;
}

}  // namespace sbpl_interface

#endif  // _ENVIRONMENT_CHAIN3D_MOVEIT_H_
