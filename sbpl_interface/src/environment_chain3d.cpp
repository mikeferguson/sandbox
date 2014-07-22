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

#include <sbpl_interface/environment_chain3d.h>

namespace sbpl_interface
{

EnvironmentChain3D::EnvironmentChain3D() :
  hash_data_(StateID2IndexMapping)
{
}

EnvironmentChain3D::~EnvironmentChain3D()
{
}

/////////////////////////////////////////////////////////////////////////////
//                      SBPL Planner Interface
/////////////////////////////////////////////////////////////////////////////

bool EnvironmentChain3D::InitializeMDPCfg(MDPConfig *MDPCfg)
{
  if (start_ && goal_)
  {
    MDPCfg->startstateid = start_->stateID;
    MDPCfg->goalstateid = goal_->stateID;
    return true;
  }
  return false;
}

bool EnvironmentChain3D::InitializeEnv(const char* sEnvFile)
{
  ROS_INFO("[env] InitializeEnv is not implemented right now.");
  return true;
}

int EnvironmentChain3D::GetFromToHeuristic(int FromStateID, int ToStateID)
{
  return getEndEffectorHeuristic(FromStateID,ToStateID);
}

int EnvironmentChain3D::GetGoalHeuristic(int stateID)
{
  return GetFromToHeuristic(stateID, goal_->stateID);
}

int EnvironmentChain3D::GetStartHeuristic(int stateID)
{
  return GetFromToHeuristic(stateID, start_->stateID);
}

int EnvironmentChain3D::SizeofCreatedEnv()
{
  return hash_data_.state_ID_to_coord_table_.size();
}

void EnvironmentChain3D::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
  ROS_DEBUG_STREAM("PrintState Not implemented");
}

void EnvironmentChain3D::PrintEnv_Config(FILE* fOut)
{
  ROS_ERROR("ERROR in EnvChain... function: PrintEnv_Config is undefined");
  throw new SBPL_Exception();
}

void EnvironmentChain3D::GetSuccs(int source_state_ID,
                                  std::vector<int>* succ_idv,
                                  std::vector<int>* cost_v)
{
  //boost::this_thread::interruption_point();
  ros::WallTime expansion_start_time = ros::WallTime::now();

  succ_idv->clear();
  cost_v->clear();

  // TODO: clean these up?
  // From environment_robarm3d.cpp -- //goal state should be absorbing
  if (source_state_ID == goal_->stateID)
  {
    ROS_WARN_STREAM("[EnvironmentChain3D::GetSuccs] Source state " << source_state_ID << " is a goal!");
    return;
  }

  if (source_state_ID > (int)hash_data_.state_ID_to_coord_table_.size()-1)
  {
    ROS_WARN_STREAM("[EnvironmentChain3D::GetSuccs] Source state " << source_state_ID << " is too large");
    return;
  }

  // Get info on source state
  EnvChain3dHashEntry* hash_entry = hash_data_.state_ID_to_coord_table_[source_state_ID];
  hash_entry->expanded = true;

  std::vector<double> source_joint_angles = hash_entry->angles;
  std::vector<double> succ_joint_angles;
  std::vector<int> succ_coord;

  for (size_t i = 0; i < prims_.size(); ++i)
  {
    double dist = params_.field_resolution *
                  static_cast<double>(getEndEffectorHeuristic(source_state_ID, goal_->stateID));

    // Get the successor state, if any
    if (!prims_[i]->getSuccessorState(source_joint_angles, &succ_joint_angles, dist))
      continue;

    // Test for collisions along path, joint limits, path constraints, etc.
    if (!isStateToStateValid(source_joint_angles, succ_joint_angles))
      continue;

    // Get end effector pose
    int xyz[3];
    if (!getEndEffectorCoord(succ_joint_angles, xyz))
      continue;

    // Get coord
    convertJointAnglesToCoord(succ_joint_angles, succ_coord);

    EnvChain3dHashEntry* succ_hash_entry = NULL;
    if (isStateGoal(succ_joint_angles))
    {
      succ_hash_entry = goal_;
      // fill in results
      succ_hash_entry->coord = succ_coord;
      succ_hash_entry->angles = succ_joint_angles;
    }
    else
    {
      succ_hash_entry = hash_data_.getHashEntry(succ_coord, i);
      if (!succ_hash_entry)
        succ_hash_entry = hash_data_.addHashEntry(succ_coord, succ_joint_angles, xyz, i);
    }

    succ_idv->push_back(succ_hash_entry->stateID);
    cost_v->push_back(calculateCost(hash_entry, succ_hash_entry));
  }

  planning_statistics_.total_expansion_time_ += ros::WallTime::now()-expansion_start_time;
  planning_statistics_.total_expansions_++;
}

void EnvironmentChain3D::GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* cost_v)
{
  ROS_ERROR("ERROR in EnvChain... function: GetPreds is undefined");
  throw new SBPL_Exception();
}

bool EnvironmentChain3D::AreEquivalent(int StateID1, int StateID2)
{
  ROS_ERROR("ERROR in EnvChain... function: AreEquivalent is undefined");
  throw new SBPL_Exception();
}

void EnvironmentChain3D::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
  ROS_ERROR("ERROR in EnvChain..function: SetAllActionsandOutcomes is undefined");
  throw new SBPL_Exception();
}

void EnvironmentChain3D::SetAllPreds(CMDPSTATE* state)
{
  ROS_ERROR("ERROR in EnvChain... function: SetAllPreds is undefined");
  throw new SBPL_Exception();
}

void EnvironmentChain3D::addMotionPrimitive(MotionPrimitivePtr& mp)
{
  prims_.push_back(mp);
}

bool EnvironmentChain3D::isStateToStateValid(const std::vector<double>& start,
                                             const std::vector<double>& end)
{
  ROS_WARN("EnvironmentChain3D has no validity function defined -- no collision checking will occur");
  return true;
}

bool EnvironmentChain3D::isStateGoal(const std::vector<double>& angles)
{
  ROS_ERROR("EnvironmentChain3D has no isStateGoal defined -- you are never going to get to the goal!");
  throw new SBPL_Exception();
}

bool EnvironmentChain3D::getEndEffectorCoord(const std::vector<double>& angles, int * xyz)
{
  ROS_WARN("EnvironmentChain3D has no getEndEffectorCoord defined");
  return false;
}

int EnvironmentChain3D::calculateCost(EnvChain3dHashEntry* HashEntry1, EnvChain3dHashEntry* HashEntry2)
{
  // TODO: apparently this is what both sbpl_manipulation and the former arm_nav/moveit interfaces do...
  return 1;
}

int EnvironmentChain3D::getEndEffectorHeuristic(int FromStateID, int ToStateID)
{
  EnvChain3dHashEntry* from = hash_data_.state_ID_to_coord_table_[FromStateID];
  return getEndEffectorHeuristic(from->xyz[0], from->xyz[1], from->xyz[2]);
}

int EnvironmentChain3D::getEndEffectorHeuristic(int x, int y, int z) const
{
  ROS_ERROR("EnvironmentChain3D has no getEndEffectorHeuristic(x,y,z) defined.");
  throw new SBPL_Exception();
}

}  // namespace sbpl_interface
