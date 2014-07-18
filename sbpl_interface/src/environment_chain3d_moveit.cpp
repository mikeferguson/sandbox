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

/** \Author: Benjamin Cohen, E. Gil Jones, Michael Ferguson **/

#include <Eigen/Core>
#include <sbpl_interface/environment_chain3d_moveit.h>
//#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>

/*
 * Notes:
 *  * not using distance field for anything except constructing BFS.
 *    in the future, distance field should be used for collision checking as well.
 */

namespace sbpl_interface
{

EnvironmentChain3DMoveIt::EnvironmentChain3DMoveIt(const planning_scene::PlanningSceneConstPtr& planning_scene) :
  EnvironmentChain3D(),
  planning_scene_(planning_scene),
  state_(new robot_state::RobotState(planning_scene->getCurrentState())),
  goal_constraint_set_(planning_scene->getRobotModel()),
  path_constraint_set_(planning_scene->getRobotModel()),
  bfs_(NULL)
{
}

EnvironmentChain3DMoveIt::~EnvironmentChain3DMoveIt()
{
  if (bfs_ != NULL)
    delete bfs_;
}

bool EnvironmentChain3DMoveIt::setupForMotionPlan(
   const planning_scene::PlanningSceneConstPtr& planning_scene,
   const moveit_msgs::MotionPlanRequest &mreq,
   moveit_msgs::MotionPlanResponse& mres)
   // TODO: add params?
{
  ROS_INFO("Setting up for motion planning!");

  // Setup data structs
  planning_scene_ = planning_scene;
  planning_group_ = mreq.group_name;
  joint_model_group_ = state_->getJointModelGroup(planning_group_);
  tip_link_model_ = state_->getLinkModel(joint_model_group_->getLinkModelNames().back());

  // Local copy of current state
  std::vector<double> start_joint_values;
  moveit::core::robotStateMsgToRobotState(mreq.start_state, *state_);
  state_->copyJointGroupPositions(planning_group_, start_joint_values);
  state_->update(); // make sure joint values aren't dirty

  // Print out the starting joint angles
  std::stringstream dbg_ss;
  dbg_ss.str("");
  for (size_t i=0; i < start_joint_values.size(); ++i)
    dbg_ss << start_joint_values[i] << " ";
  ROS_WARN_STREAM("[Start angles] " << dbg_ss.str());

  // Check start state for collision
  collision_detection::CollisionRequest creq;
  collision_detection::CollisionResult cres;
  creq.group_name = planning_group_;
  planning_scene->checkCollision(creq, cres, *state_, planning_scene_->getAllowedCollisionMatrix());
  if (cres.collision)
  {
    ROS_WARN_STREAM("Start state is in collision. Can't plan");
    mres.error_code.val = moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
    return false;
  }

  // TODO: create distance field and update it
  // TODO: setup BFS walls from distance field
  
  // TODO: setup motion primitives (based on params?)

  // Setup start position in discrete space
  std::vector<int> start_coords;
  convertJointAnglesToCoord(start_joint_values, start_coords);
  dbg_ss.str("");
  for (size_t i=0; i<start_coords.size(); ++i)
    dbg_ss << start_coords[i] << " ";
  ROS_WARN_STREAM("[Start coords] " << dbg_ss.str());

  int start_xyz[3];
  if (!getEndEffectorXYZ(start_joint_values, start_xyz))
  {
    ROS_ERROR("Bad start pose");
    mres.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }
  start_ = hash_data_.addHashEntry(start_coords,
                                   start_joint_values,
                                   start_xyz,
                                   0);

  // Setup goal position in discrete space
  for (size_t i = 0; i < mreq.goal_constraints[0].joint_constraints.size(); ++i)
  {
    state_->setJointPositions(mreq.goal_constraints[0].joint_constraints[i].joint_name,
                              &mreq.goal_constraints[0].joint_constraints[i].position);
  }
  state_->update();

  // To be used later in constructing the goal
  std::vector<double> goal_joint_values;
  std::vector<int> goal_coords;
  int goal_xyz[3];

  // Check goal state (if any) for collisions
  if (mreq.goal_constraints[0].joint_constraints.size() > 0)
  {
    planning_scene->checkCollision(creq, cres, *state_, planning_scene_->getAllowedCollisionMatrix());
    if (cres.collision)
    {
      ROS_WARN_STREAM("Goal state is in collision.  Can't plan");
      mres.error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
      return false;
    }

    // Collision free goal, generate the data we need for the planner
    state_->copyJointGroupPositions(planning_group_, goal_joint_values);
    state_->update();

    convertJointAnglesToCoord(goal_joint_values, goal_coords);
    if (!getEndEffectorXYZ(goal_joint_values, goal_xyz))
    {
      ROS_ERROR("Bad goal pose");
      mres.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      return false;
    }

    // Post the generated data
    dbg_ss.str("");
    for (size_t i=0; i<goal_joint_values.size(); ++i)
      dbg_ss << goal_joint_values[i] << " ";
    ROS_WARN_STREAM("[Goal angles] " << dbg_ss.str());
    dbg_ss.str("");
    for (size_t i=0; i<goal_coords.size(); ++i)
      dbg_ss << goal_coords[i] << " ";
    ROS_WARN_STREAM("[Goal coords] " << dbg_ss.str());
  
  }
  else
  {
    ROS_WARN("Goal does not have joint constraints.");

    // Fill in joint data (it doesn't matter)
    goal_joint_values.resize(start_joint_values.size());
    goal_coords.resize(start_joint_values.size());

    // TODO: fill in goal_xyz with the values from the other constraints so that heuristic will work
    ROS_ERROR("Only joint constraints are currently allowed!");
    return false;
  }

  std::vector<std::string> goal_dofs = state_->getJointModelGroup(planning_group_)->getActiveJointModelNames();
  assert(goal_dofs.size() == start_joint_values.size());
  assert(goal_dofs.size() == goal_joint_values.size());

  bfs_->run(goal_xyz[0], goal_xyz[1], goal_xyz[2]);

  // Setup goal constraints
  goal_constraint_set_.clear();
  goal_constraint_set_.add(mreq.goal_constraints[0], planning_scene_->getTransforms());
  goal_ = hash_data_.addHashEntry(goal_coords, goal_joint_values, goal_xyz, 0);

  // Setup path constraints
  path_constraint_set_.clear();
  path_constraint_set_.add(mreq.path_constraints, planning_scene_->getTransforms());

  return true;
}

bool EnvironmentChain3DMoveIt::isStateToStateValid(const std::vector<double>& start,
                                                   const std::vector<double>& end)
{
  // Update robot_state
  state_->setJointGroupPositions(joint_model_group_, start);
  state_->update();

  // Ensure path constraints
  kinematic_constraints::ConstraintEvaluationResult con_res = path_constraint_set_.decide(*state_);
  if (!con_res.satisfied)
    return false;

  ros::WallTime before_coll = ros::WallTime::now();

  // Ensure collision free
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.group_name = planning_group_;
  planning_scene_->checkCollision(req, res, *state_);

  // Update profiling
  ros::WallDuration dur(ros::WallTime::now()-before_coll);
  planning_statistics_.total_coll_check_time_ += dur;
  planning_statistics_.coll_checks_++;

  if (res.collision)
    return false;

  return true;
}

bool EnvironmentChain3DMoveIt::isStateGoal(const std::vector<double>& angles)
{
  // Update robot_state
  state_->setJointGroupPositions(joint_model_group_, angles);
  state_->update();

  // Are goal constraints met?
  kinematic_constraints::ConstraintEvaluationResult con_res = goal_constraint_set_.decide(*state_);
  return con_res.satisfied;
}

bool EnvironmentChain3DMoveIt::getEndEffectorXYZ(const std::vector<double>& angles, int * xyz)
{
  // Update robot_state
  state_->setJointGroupPositions(joint_model_group_, angles);
  state_->update();

  // Get pose of end effector
  Eigen::Affine3d pose = state_->getGlobalLinkTransform(tip_link_model_);

  // TODO convert pose into xyz in BFS grid
  

  return true;
}

int EnvironmentChain3DMoveIt::getEndEffectorHeuristic(int FromStateID, int ToStateID)
{
  //boost::this_thread::interruption_point();
  EnvChain3dHashEntry* from_hash_entry = hash_data_.state_ID_to_coord_table_[FromStateID];

  if (1)//use_bfs_)
  {
    // Return the BFS cost to goal
    return int(bfs_->getDistance(from_hash_entry->xyz[0],
                                 from_hash_entry->xyz[1],
                                 from_hash_entry->xyz[2])) /** cost_per_cell_ TODO */;
  }
  else
  {
    // Return euclidean distance to goal
    double x, y, z;
    // TODO convert xyz into continuous world x, y, z to compare against goal?
    // TODO maybe we can just do this in discrete space?
    //grid_->gridToWorld(FromHashEntry->xyz[0],FromHashEntry->xyz[1],FromHashEntry->xyz[2], x, y, z);
    //return getEuclideanDistance(x, y, z, pdata_.goal.pose[0], pdata_.goal.pose[1], pdata_.goal.pose[2]) * prm_->cost_per_meter_;
  }
}    

}  // namespace sbpl_interface
