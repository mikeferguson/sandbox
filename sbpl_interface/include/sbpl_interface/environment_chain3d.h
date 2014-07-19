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

#ifndef _ENVIRONMENT_CHAIN3D_H_
#define _ENVIRONMENT_CHAIN3D_H_

#include <ros/ros.h>
#include <angles/angles.h>
#include <sbpl/headers.h>
#include <sbpl_interface/motion_primitives.h>
#include <sbpl_interface/sbpl_planning_params.h>

namespace sbpl_interface
{

static unsigned int HASH_TABLE_SIZE = 32*1024;

static inline unsigned int intHash(unsigned int key)
{
  key += (key << 12);
  key ^= (key >> 22);
  key += (key << 4);
  key ^= (key >> 9);
  key += (key << 10);
  key ^= (key >> 2);
  key += (key << 7);
  key ^= (key >> 12);
  return key;
}

struct EnvChain3dHashEntry
{
  bool expanded;
  int stateID;                  /// hash entry ID number
  int action;                   /// which action in the list was required to get here
  int xyz[3];                   /// tip link coordinates in discrete space
  std::vector<int> coord;       /// joint positions in the discretization
  std::vector<double> angles;   /// joint positions in continuous space
};

/** \brief Holds the actual HashEntry instances */
struct Env3dHashData
{
  Env3dHashData(std::vector<int*>& state_ID_to_index_mapping) :
    state_ID_to_index_mapping_(state_ID_to_index_mapping),
    hash_table_size_(HASH_TABLE_SIZE)
  {
    coord_to_state_ID_table_.resize(hash_table_size_);
  }

  ~Env3dHashData()
  {
    for (size_t i = 0; i < state_ID_to_coord_table_.size(); ++i)
      delete state_ID_to_coord_table_[i];
  }

  unsigned int getHashBin(const std::vector<int>& coord)
  {
    unsigned int val = 0;

    for (size_t i = 0; i < coord.size(); ++i)
      val += intHash(coord[i]) << i;

    return intHash(val) & (hash_table_size_-1);
  }

  EnvChain3dHashEntry* addHashEntry(const std::vector<int>& coord,
                                    const std::vector<double>& angles,
                                    const int(&xyz)[3],
                                    int action)
  {
    EnvChain3dHashEntry* new_hash_entry = new EnvChain3dHashEntry();
    new_hash_entry->stateID = state_ID_to_coord_table_.size();
    new_hash_entry->coord = coord;
    new_hash_entry->angles = angles;
    memcpy(new_hash_entry->xyz, xyz, sizeof(int)*3);
    new_hash_entry->action = action;
    state_ID_to_coord_table_.push_back(new_hash_entry);

    unsigned int bin = getHashBin(coord);
    coord_to_state_ID_table_[bin].push_back(new_hash_entry);

    // have to do for DiscreteSpaceInformation
    // insert into and initialize the mappings
    int* entry = new int [NUMOFINDICES_STATEID2IND];
    memset(entry, -1, NUMOFINDICES_STATEID2IND*sizeof(int));
    state_ID_to_index_mapping_.push_back(entry);
    if (new_hash_entry->stateID != (int)state_ID_to_index_mapping_.size()-1)
      ROS_ERROR_STREAM("Size mismatch between state mappings " << new_hash_entry->stateID << " " << state_ID_to_index_mapping_.size());

    return new_hash_entry;
  }

  EnvChain3dHashEntry* getHashEntry(const std::vector<int> &coord,
                                    int action)
  {
    unsigned int bin = getHashBin(coord);
    for (size_t i = 0; i < coord_to_state_ID_table_[bin].size(); ++i)
    {
      if (coord_to_state_ID_table_[bin][i]->coord == coord)
        return coord_to_state_ID_table_[bin][i];
    }
    return NULL;
  }

  /// internal data from DiscreteSpaceInformation
  std::vector<int*>& state_ID_to_index_mapping_;

  /// maps from coords to vector of stateID
  std::vector<std::vector<EnvChain3dHashEntry*> > coord_to_state_ID_table_;

  /// vector that maps from stateID to coords
  std::vector<EnvChain3dHashEntry*> state_ID_to_coord_table_;

  unsigned int hash_table_size_;
};

/** \brief Statistics to be collected */
struct PlanningStatistics
{
  PlanningStatistics() :
    total_expansions_(0),
    coll_checks_(0)
  {
  }

  double distance_field_percent_occupied_;
  ros::WallDuration distance_field_setup_time_;
  ros::WallDuration heuristic_setup_time_;
  ros::WallDuration total_setup_time_;

  ros::WallDuration heuristic_run_time_;

  unsigned int total_expansions_;
  ros::WallDuration total_expansion_time_;

  unsigned int coll_checks_;
  ros::WallDuration total_coll_check_time_;

  ros::WallDuration total_planning_time_;
  ros::WallDuration shortcutting_time_;
  ros::WallDuration total_time_;

  void print()
  {
    ROS_INFO_STREAM("Distance Field Setup Time: " << distance_field_setup_time_.toSec() << ". " <<
                    "Occupied: " << distance_field_percent_occupied_ << "%");
    ROS_INFO_STREAM("Heuristic Setup Time: " << heuristic_setup_time_.toSec());
    ROS_INFO_STREAM("Heuristic Run Time: " << heuristic_run_time_.toSec());
    ROS_INFO_STREAM("Expansions: " << total_expansions_ << ". " <<
                    "Average Time: " << total_expansion_time_.toSec()/static_cast<double>(total_expansions_) << "s " <<
                    "Freq: " << 1.0/(total_expansion_time_.toSec()/static_cast<double>(total_expansions_)) << "hz");
    ROS_INFO_STREAM("Total collision checks " << coll_checks_ << ". " <<
                    "Freq: " << 1.0/(total_coll_check_time_.toSec()/static_cast<double>(coll_checks_)) << "hz");
    ROS_INFO_STREAM("Total Planning Time: " << total_planning_time_.toSec());
    ROS_INFO_STREAM("Total Shortcut Time: " << shortcutting_time_.toSec());
    ROS_INFO_STREAM("Total Time: " << total_time_.toSec());
  }
};

/** \brief Environment to be used when planning for a Robotic Arm using the SBPL. */
class EnvironmentChain3D: public DiscreteSpaceInformation
{
public:
  /**
   * @brief Default constructor
   */
  EnvironmentChain3D();

  /**
   * @brief Destructor
   */
  virtual ~EnvironmentChain3D();

  //
  // Pure virtual functions from DiscreteSpaceInformation
  //

  /**
   * @brief Initialize the environment from a text file
   * @param name of environment text file
   * @return true if successful, false otherwise
   */
  virtual bool InitializeEnv(const char* sEnvFile);

  /**
   * @brief Initialize the start and goal states of the MDP
   * @param always returns true...
   */
  virtual bool InitializeMDPCfg(MDPConfig *MDPCfg);

  /**
   * @brief Get the heuristic from one state to another state.
   * @param the stateID of the current state
   * @param the stateID of the goal state
   * @return h(s,s')
   */
  virtual int GetFromToHeuristic(int FromStateID, int ToStateID);

  /**
   * @brief Get the heuristic of a state to the planner's goal state.
   * @param the stateID of the current state
   * @return h(s,s_goal)
   */
  virtual int GetGoalHeuristic(int stateID);

  /**
   * @brief Get the heuristic of a state to the planner's start state.
   * @param the stateID of the current state
   * @return h(s,s_start)
   */
  virtual int GetStartHeuristic(int stateID);

  /**
   * @brief Get the successors of the desired state to be expanded.
   * Return vectors with the successors' state IDs and the cost to move
   * from the current state to that state. If the vectors return to the
   * planner empty then the search quits.
   * @param the state ID of the state to be expanded
   * @param a pointer to a vector that will be populated with the
   * successor state IDs.
   * @param a pointer to a vector that will be populated with the costs of
   * transitioning from the current state to the successor state.
   */
  virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);

  /** @brief Not defined. */
  virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);

  /** @brief Not defined. */
  virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);

  /** @brief Not defined. */
  virtual void SetAllPreds(CMDPSTATE* state);

  /**
   * @brief This function returns the number of hash entries created.
   * @return number of hash entries
   */
  virtual int SizeofCreatedEnv();

  /**
   * @brief This function prints out the state information of a state.
   * @param the state ID of the desired state
   * @param prints out a little extra information
   * @param the file pointer to print to (stdout by default)
   */
  virtual void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);

  /** @brief Not defined. */
  virtual void PrintEnv_Config(FILE* fOut);

  //
  // Overloaded virtual functions from DiscreteSpaceInformation
  //

  /**
   * @brief Check if the states with StateID1 & StateID2 are equivalent
   * based on an equivalency function with some declared tolerance.
   * @param stateID of first state
   * @param stateID of second state
   * @return true if equivalent, false otherwise
   */
  virtual bool AreEquivalent(int StateID1, int StateID2);

  /** @brief Add a motion primitive */
  virtual void addMotionPrimitive(MotionPrimitivePtr& mp);

  /** @brief Get the planning parameters */
  SBPLPlanningParams& getParams() { return params_; }

  PlanningStatistics& getPlanningStatistics() { return planning_statistics_; }

  int getStartID() { return start_->stateID; }
  int getGoalID() { return goal_->stateID; }

protected:
  /** @brief Is state valid? This should be defined by the interface. */
  virtual bool isStateToStateValid(const std::vector<double>& start,
                                   const std::vector<double>& end);

  /** @brief Is state close enough to be goal? This should be defined by the interface. */
  virtual bool isStateGoal(const std::vector<double>& angles);

  /** @brief Get the grid location of the end effector based on joint angles. */
  virtual bool getEndEffectorCoord(const std::vector<double>& angles, int * xyz);

  /** @brief Calculate the cost for moving from one state to another -- this is used in GetSuccs. */
  virtual int calculateCost(EnvChain3dHashEntry* HashEntry1, EnvChain3dHashEntry* HashEntry2);

  /** @brief The heuristic */
  virtual int getEndEffectorHeuristic(int FromStateID, int ToStateID);
  virtual int getEndEffectorHeuristic(int x, int y, int z);

  void convertJointAnglesToCoord(const std::vector<double> &angle, std::vector<int> &coord);
  void convertCoordToJointAngles(const std::vector<int> &coord, std::vector<double> &angles);

  // The hash table of states, as well as start and goal entries
  Env3dHashData hash_data_;
  EnvChain3dHashEntry* start_;
  EnvChain3dHashEntry* goal_;

  // Parameters
  SBPLPlanningParams params_;

  // The motion primitives
  std::vector<MotionPrimitivePtr> prims_;

  // Track our time and expansions
  PlanningStatistics planning_statistics_;
};

inline void EnvironmentChain3D::convertJointAnglesToCoord(const std::vector<double> &angle, std::vector<int> &coord)
{
  coord.resize(angle.size());
  for (size_t i = 0; i < angle.size(); ++i)
  {
    double pos_angle = angles::normalize_angle_positive(angle[i]);
    coord[i] = (int)((pos_angle + params_.angle_discretization*0.5)/params_.angle_discretization);
  }
}

inline void EnvironmentChain3D::convertCoordToJointAngles(const std::vector<int> &coord, std::vector<double> &angle)
{
  angle.resize(coord.size());
  for (size_t i = 0; i < coord.size(); ++i)
    angle[i] = coord[i]*params_.angle_discretization;
}

}  // namespace sbpl_interface

#endif  // _ENVIRONMENT_CHAIN3D_H_
