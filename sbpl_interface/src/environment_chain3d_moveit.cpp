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
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/link_model.h>
#include <eigen_conversions/eigen_msg.h>

namespace sbpl_interface
{

inline double getEuclideanDistance(double x1, double y1, double z1, double x2, double y2, double z2)
{
  return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));
}

EnvironmentChain3DMoveIt::EnvironmentChain3DMoveIt() :
  EnvironmentChain3D(),
  goal_constraint_set_(NULL),
  path_constraint_set_(NULL),
  bfs_(NULL),
  field_(NULL)
{
}

EnvironmentChain3DMoveIt::~EnvironmentChain3DMoveIt()
{
  if (bfs_ != NULL)
    delete bfs_;
  if (field_ != NULL)
    delete field_;
}

bool EnvironmentChain3DMoveIt::setupForMotionPlan(
   const planning_scene::PlanningSceneConstPtr& planning_scene,
   const moveit_msgs::MotionPlanRequest &mreq,
   moveit_msgs::MotionPlanResponse& mres,
   SBPLPlanningParams& params)
{
  ros::WallTime setup_start = ros::WallTime::now();

  // Setup data structs
  planning_scene_ = planning_scene;
  planning_group_ = mreq.group_name;
  state_.reset(new robot_state::RobotState(planning_scene->getCurrentState()));
  joint_model_group_ = state_->getJointModelGroup(planning_group_);
  tip_link_model_ = state_->getLinkModel(joint_model_group_->getLinkModelNames().back());
  params_ = params;

  // Local copy of current state
  std::vector<double> start_joint_values;
  moveit::core::robotStateMsgToRobotState(mreq.start_state, *state_);
  state_->copyJointGroupPositions(planning_group_, start_joint_values);
  state_->update();  // make sure joint values aren't dirty

  // Print out the starting joint angles
  std::stringstream dbg_ss;
  dbg_ss.str("");
  for (size_t i=0; i < start_joint_values.size(); ++i)
    dbg_ss << start_joint_values[i] << " ";
  ROS_DEBUG_STREAM_NAMED("sbpl", "[Start angles] " << dbg_ss.str());

  // Check start state for collision
  collision_detection::CollisionRequest creq;
  collision_detection::CollisionResult cres;
  creq.group_name = planning_group_;
  planning_scene->checkCollision(creq, cres, *state_, planning_scene_->getAllowedCollisionMatrix());
  if (cres.collision)
  {
    ROS_ERROR_STREAM_NAMED("sbpl", "Start state is in collision. Can't plan");
    mres.error_code.val = moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
    return false;
  }

  // Setup start position in discrete space
  std::vector<int> start_coords;
  convertJointAnglesToCoord(start_joint_values, start_coords);
  dbg_ss.str("");
  for (size_t i=0; i<start_coords.size(); ++i)
    dbg_ss << start_coords[i] << " ";
  ROS_DEBUG_STREAM_NAMED("sbpl", "[Start coords] " << dbg_ss.str());

  int start_xyz[3];
  if (!getEndEffectorCoord(start_joint_values, start_xyz))
  {
    ROS_ERROR_NAMED("sbpl", "Bad start pose");
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
      ROS_ERROR_STREAM_NAMED("sbpl", "Goal state is in collision.  Can't plan");
      mres.error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
      return false;
    }

    // Collision free goal, generate the data we need for the planner
    state_->copyJointGroupPositions(planning_group_, goal_joint_values);
    state_->update();

    convertJointAnglesToCoord(goal_joint_values, goal_coords);
    if (!getEndEffectorCoord(goal_joint_values, goal_xyz))
    {
      ROS_ERROR_NAMED("sbpl", "Bad goal pose");
      mres.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      return false;
    }

    // Post the generated data
    dbg_ss.str("");
    for (size_t i=0; i<goal_joint_values.size(); ++i)
      dbg_ss << goal_joint_values[i] << " ";
    ROS_DEBUG_STREAM_NAMED("sbpl", "[Goal angles] " << dbg_ss.str());
    dbg_ss.str("");
    for (size_t i=0; i<goal_coords.size(); ++i)
      dbg_ss << goal_coords[i] << " ";
    ROS_DEBUG_STREAM_NAMED("sbpl", "[Goal coords] " << dbg_ss.str());

    // Planning in joint space -- should add a snap to joints primitive
    if (params_.primitives.use_joint_snap)
    {
      MotionPrimitivePtr snap(new SnapToJointMotionPrimitive(goal_joint_values,
                                                             params_.primitives.joint_snap_thresh));
      addMotionPrimitive(snap);
      ROS_DEBUG_NAMED("sbpl", "Added joint snap motion prim");
    }
  }
  else
  {
    // Fill in joint data (it doesn't matter)
    goal_joint_values.resize(start_joint_values.size());
    goal_coords.resize(start_joint_values.size());

    // Fill in goal_xyz with the values from position constraints so that heuristic will work
    if (mreq.goal_constraints[0].position_constraints.size() > 0 &&
        mreq.goal_constraints[0].orientation_constraints.size() > 0 &&
        mreq.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses.size() > 0)
    {
      //ROS_WARN("Planner assumes that pose is in model frame and that link is set to end effector link.");
      geometry_msgs::Pose pose = mreq.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0];
      continuousXYZtoDiscreteXYZ(pose.position.x, pose.position.y, pose.position.z, goal_xyz[0], goal_xyz[1], goal_xyz[2]);
    }
    else
    {
      ROS_ERROR_NAMED("sbpl", "No joint or position constraints -- cannot plan!");
      return false;
    }

    dbg_ss.str("");
    for (size_t i=0; i<3; ++i)
      dbg_ss << goal_xyz[i] << " ";
    ROS_DEBUG_STREAM_NAMED("sbpl", "[Goal] " << dbg_ss.str());

    // Add snap to xyzrpy primitive
    if (params_.primitives.use_xyzrpy_snap)
    {
      MotionPrimitivePtr snap(new SnapToXYZRPYMotionPrimitive(mreq.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0],
                                                              mreq.goal_constraints[0].orientation_constraints[0].orientation,
                                                              state_, joint_model_group_,
                                                              params_.primitives.xyzrpy_snap_thresh));
      addMotionPrimitive(snap);
      ROS_DEBUG_NAMED("sbpl", "Added xyzrpy snap motion prim");
    }
  }

  std::vector<std::string> goal_dofs = state_->getJointModelGroup(planning_group_)->getActiveJointModelNames();
  assert(goal_dofs.size() == start_joint_values.size());
  assert(goal_dofs.size() == goal_joint_values.size());

  if (params_.use_bfs)
  {
    ROS_DEBUG_NAMED("sbpl", "Setting up to use BFS.");

    // Create distance field
    ros::WallTime distance_start = ros::WallTime::now();
    field_ = new distance_field::PropagationDistanceField(params_.field_x, params.field_y, params_.field_z,
                                                          params_.field_resolution,
                                                          params_.field_origin_x, params_.field_origin_y, params.field_origin_z,
                                                          params_.field_z /* max distance, all cells initialize to this */);
    // TODO This could be massively improved (especially if we switch to some variation of the hybrid distance field)
    // Add objects to distance field from planning scene
    collision_detection::WorldConstPtr world = planning_scene_->getWorld();
    std::vector<std::string> objects = world->getObjectIds();
    for (size_t i = 0; i < objects.size(); ++i)
    {
      collision_detection::World::ObjectConstPtr obj = world->getObject(objects[i]);
      if (obj->shapes_.size() == 0)
        continue;
      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(obj->shape_poses_[0], pose);
      field_->addShapeToField(obj->shapes_[0].get(), pose);
    }
    // Add field links to distance field
    for (size_t i = 0; i < params_.field_links.size(); ++i)
    {
      const moveit::core::LinkModel* link = state_->getLinkModel(params_.field_links[i]);
      const Eigen::Affine3d global_t = state_->getGlobalLinkTransform(link->getName());
      const std::vector<shapes::ShapeConstPtr> shapes = link->getShapes();
      const EigenSTL::vector_Affine3d shape_poses = link->getCollisionOriginTransforms();
      for (size_t j = 0; j < shapes.size(); ++j)
      {
        Eigen::Affine3d t = global_t * shape_poses[j];
        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(t, pose);
        field_->addShapeToField(shapes[j].get(), pose);
      }
    }
    planning_statistics_.distance_field_setup_time_ = ros::WallTime::now() - distance_start;

    // Setup BFS
    bfs_ = new BFS_3D(field_->getXNumCells(), field_->getYNumCells(), field_->getZNumCells());
    // Push obstacles from distance field
    ros::WallTime heuristic_start = ros::WallTime::now();
    int walls = fillBFSfromField(field_, bfs_, params_);
    planning_statistics_.heuristic_setup_time_ = ros::WallTime::now() - heuristic_start;
    planning_statistics_.distance_field_percent_occupied_ = double(walls)/double(field_->getXNumCells()*field_->getYNumCells()*field_->getZNumCells());
    // Run BFS, have it update planning_stats when done
    bfs_->run(goal_xyz[0], goal_xyz[1], goal_xyz[2], &planning_statistics_.heuristic_run_time_);
  }

  // Setup goal constraints
  if (goal_constraint_set_ == NULL)
    goal_constraint_set_ = new kinematic_constraints::KinematicConstraintSet(planning_scene_->getRobotModel());
  goal_constraint_set_->clear();
  goal_constraint_set_->add(mreq.goal_constraints[0], planning_scene_->getTransforms());
  goal_ = hash_data_.addHashEntry(goal_coords, goal_joint_values, goal_xyz, 0);

  // Setup path constraints
  if (path_constraint_set_ == NULL)
    path_constraint_set_ = new kinematic_constraints::KinematicConstraintSet(planning_scene_->getRobotModel());
  path_constraint_set_->clear();
  path_constraint_set_->add(mreq.path_constraints, planning_scene_->getTransforms());

  planning_statistics_.total_setup_time_ = ros::WallTime::now() - setup_start;
  ROS_DEBUG_NAMED("sbpl", "Setup for SBPL motion planning is complete!");
  return true;
}

bool EnvironmentChain3DMoveIt::populateTrajectoryFromStateIDSequence(
    const std::vector<int>& state_ids,
    trajectory_msgs::JointTrajectory& traj) const
{
  traj.joint_names = joint_model_group_->getActiveJointModelNames();
  traj.points.resize(state_ids.size());
  for (size_t i = 0; i < state_ids.size(); ++i)
  {
    if (state_ids[i] > (int) hash_data_.state_ID_to_coord_table_.size()-1)
      return false;
    traj.points[i].positions = hash_data_.state_ID_to_coord_table_[state_ids[i]]->angles;
  }
  return true;
}

bool EnvironmentChain3DMoveIt::isStateToStateValid(const std::vector<double>& start,
                                                   const std::vector<double>& end)
{
  // Update robot_state
  state_->setJointGroupPositions(joint_model_group_, end);
  state_->update();

  // Ensure joint limits are not exceeded
  if (!state_->satisfiesBounds(joint_model_group_))
    return false;

  // Ensure path constraints
  kinematic_constraints::ConstraintEvaluationResult con_res = path_constraint_set_->decide(*state_);
  if (!con_res.satisfied)
    return false;

  // Ensure endpoint is collision free
  if (!isCollisionFree(state_))
    return false;

  // Ensure collision free along path if start/end are far apart
  std::vector<std::vector<double> > unused;
  return interpolateAndCollisionCheck(start, end, unused);
}

bool EnvironmentChain3DMoveIt::isStateGoal(const std::vector<double>& angles)
{
  // Update robot_state
  state_->setJointGroupPositions(joint_model_group_, angles);
  state_->update();

  // Are goal constraints met?
  kinematic_constraints::ConstraintEvaluationResult con_res = goal_constraint_set_->decide(*state_);
  return con_res.satisfied;
}

bool EnvironmentChain3DMoveIt::getEndEffectorCoord(const std::vector<double>& angles, int * xyz)
{
  // Update robot_state
  state_->setJointGroupPositions(joint_model_group_, angles);
  state_->update();

  // Get pose of end effector
  Eigen::Affine3d pose = state_->getGlobalLinkTransform(tip_link_model_);

  // Convert pose into xyz in BFS grid
  return continuousXYZtoDiscreteXYZ(pose.translation().x(),
                                    pose.translation().y(),
                                    pose.translation().z(),
                                    xyz[0],
                                    xyz[1],
                                    xyz[2]);
}

bool EnvironmentChain3DMoveIt::continuousXYZtoDiscreteXYZ(
  const double X, const double Y, const double Z,
  int& x, int& y, int& z)
{
  x = (X - params_.field_origin_x) / params_.field_resolution;
  y = (Y - params_.field_origin_y) / params_.field_resolution;
  z = (Z - params_.field_origin_z) / params_.field_resolution;
  // TODO: should we check limits of the field?
  return true;
}

int EnvironmentChain3DMoveIt::calculateCost(EnvChain3dHashEntry* from, EnvChain3dHashEntry* to)
{
  // Distance to obstacle in state s'
  double dist_to_obs = field_->getDistance(to->xyz[0], to->xyz[1], to->xyz[2]);

  // cost_cell(s')
  int cost_cell = 1;
  if (dist_to_obs < params_.max_obstacle_dist)
    cost_cell = static_cast<int>((params_.max_obstacle_dist - dist_to_obs) * params_.cost_obstacle_dist);

  // cost_action(s,s')
  int cost_action = getEndEffectorHeuristic(from->xyz[0], from->xyz[1], from->xyz[2]) -
                    getEndEffectorHeuristic(to->xyz[0], to->xyz[1], to->xyz[2]);
  if (cost_action < 0)
    cost_action = 1;

  // cost_smooth(s,s')
  int cost_smooth = 1;
  if (from->action == to->action)
    cost_smooth = 0;

  return params_.w_cell * cost_cell +
         params_.w_action * cost_action +
         params_.w_smooth * cost_smooth;
}

int EnvironmentChain3DMoveIt::getEndEffectorHeuristic(int x, int y, int z) const
{
  boost::this_thread::interruption_point();
  if (params_.use_bfs)
  {
    // Return the BFS cost to goal
    return static_cast<int>(bfs_->getDistance(x,y,z)) * params_.cost_per_cell;
  }
  else
  {
    // Return euclidean distance to goal
    double dist = getEuclideanDistance(x, y, z,
                                       goal_->xyz[0], goal_->xyz[1], goal_->xyz[2]);
    return static_cast<int>(dist * params_.field_resolution * params_.cost_per_meter);
  }
}

// helper for interpolateAndCollisionCheck
int getJointDistanceIntegerMax(const std::vector<double>& angles1,
                               const std::vector<double>& angles2,
                               double delta)
{
  if (angles1.size() != angles2.size())
  {
    ROS_ERROR_NAMED("sbpl", "getJointDistanceIntegerMax: Angles aren't the same size!");
    return INT_MAX;
  }

  int max_dist = 0;
  for (size_t i = 0; i < angles1.size(); i++)
  {
    int dist = floor(fabs(angles2[i]-angles1[i])/delta);
    if (i == 4 || i == 6)
    {
      // Hack -- continuous joints TODO: make this proper
      dist = floor(fabs(angles::shortest_angular_distance(angles1[i],angles2[i]))/delta);
    }
    if (dist > max_dist)
      max_dist = dist;
  }
  return max_dist;
}

bool EnvironmentChain3DMoveIt::isCollisionFree(robot_state::RobotStatePtr rs)
{
  ros::WallTime before = ros::WallTime::now();

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  req.group_name = planning_group_;

  planning_scene_->checkCollision(req, res, *rs);
  planning_statistics_.coll_checks_++;

  planning_statistics_.total_coll_check_time_ += ros::WallTime::now() - before;

  if (res.collision)
    return false;

  return true;
}

bool EnvironmentChain3DMoveIt::interpolateAndCollisionCheck(
    const std::vector<double> angles1,
    const std::vector<double> angles2,
    std::vector<std::vector<double> >& state_values)
{
  state_values.clear();

  robot_state::RobotStatePtr rs_1(new robot_state::RobotState(*state_));
  robot_state::RobotStatePtr rs_2(new robot_state::RobotState(*state_));
  robot_state::RobotStatePtr rs_temp(new robot_state::RobotState(*state_));

  rs_1->setJointGroupPositions(planning_group_, angles1);
  rs_2->setJointGroupPositions(planning_group_, angles2);
  rs_temp->setJointGroupPositions(planning_group_, angles1);

  collision_detection::CollisionRequest req;
  req.group_name = planning_group_;

  int maximum_moves = getJointDistanceIntegerMax(angles1, angles2, params_.interpolation_distance);

  // Don't collision check endpoints
  for (int i = 1; i < maximum_moves-1; ++i)
  {
    rs_1->interpolate(*rs_2,
                      (1.0/static_cast<double>(maximum_moves))*i,
                      *rs_temp);
    // interpolation puts the result into a dirty state
    rs_temp->update();

    if (!isCollisionFree(rs_temp))
      return false;

    state_values.resize(state_values.size()+1);
    rs_temp->copyJointGroupPositions(planning_group_, state_values.back());
  }

  state_values.push_back(angles2);
  return true;
}

void EnvironmentChain3DMoveIt::attemptShortcut(const trajectory_msgs::JointTrajectory& traj_in,
                                               trajectory_msgs::JointTrajectory& traj_out)
{
  ros::WallTime start = ros::WallTime::now();
  unsigned int last_point_ind = 0;
  unsigned int current_point_ind = 1;
  unsigned int last_good_start_ind = 0;
  unsigned int last_good_end_ind = 1;

  traj_out.joint_names = traj_in.joint_names;

  if (traj_in.points.size() == 1)
  {
    traj_out = traj_in;
    return;
  }

  traj_out.points.clear();
  traj_out.points.push_back(traj_in.points.front());
  std::vector< std::vector<double> > last_good_segment_values;

  // Try to jump some points.
  while (1)
  {
    const trajectory_msgs::JointTrajectoryPoint& start_point = traj_in.points[last_point_ind];
    const trajectory_msgs::JointTrajectoryPoint& end_point = traj_in.points[current_point_ind];
    std::vector< std::vector<double> > segment_values;

    // if we can go from start to end then keep going
    if (interpolateAndCollisionCheck(start_point.positions,
                                     end_point.positions,
                                     segment_values))
    {
      last_good_start_ind = last_point_ind;
      last_good_end_ind = current_point_ind;
      last_good_segment_values = segment_values;
      current_point_ind++;
    }
    else
    {
      if (last_good_end_ind-last_good_start_ind == 1)
      {
        // start and end are adjacent, copy the end in
        traj_out.points.push_back(traj_in.points[last_good_end_ind]);
      }
      else
      {
        // have interpolated points to copy in
        for (size_t i = 0; i < last_good_segment_values.size(); ++i)
        {
          trajectory_msgs::JointTrajectoryPoint jtp;
          jtp.positions = last_good_segment_values[i];
          traj_out.points.push_back(jtp);
        }
      }
      last_good_start_ind = last_good_end_ind;
      last_point_ind = last_good_end_ind;
      current_point_ind = last_good_end_ind+1;
      last_good_segment_values.clear();
    }

    if (current_point_ind >= traj_in.points.size())
    {
      // done parsing trajectory, clean up
      if (last_good_segment_values.size() > 0)
      {
        for (size_t i = 0; i < last_good_segment_values.size(); ++i)
        {
          trajectory_msgs::JointTrajectoryPoint jtp;
          jtp.positions = last_good_segment_values[i];
          traj_out.points.push_back(jtp);
        }
      }
      traj_out.points.push_back(traj_in.points.back());
      break;
    }
  }

  planning_statistics_.shortcutting_time_ = ros::WallTime::now() - start;
}

void EnvironmentChain3DMoveIt::getExpandedStates(std::vector< std::vector<double> >& states) const
{
  states.clear();

  int xy = static_cast<int>(params_.field_x/params_.field_resolution * params_.field_y/params_.field_resolution);
  int dx = static_cast<int>(params_.field_x/params_.field_resolution);

  // compute which cells are expanded
  for (size_t i = 0; i < hash_data_.state_ID_to_coord_table_.size(); ++i)
  {
    EnvChain3dHashEntry* entry = hash_data_.state_ID_to_coord_table_[i];
    if (entry->expanded)
    {
      std::vector<double> data(4, 0.0);
      data[0] = (entry->xyz[0] * params_.field_resolution) + params_.field_origin_x;
      data[1] = (entry->xyz[1] * params_.field_resolution) + params_.field_origin_y;
      data[2] = (entry->xyz[2] * params_.field_resolution) + params_.field_origin_z;

      int idx = entry->xyz[2] * xy;
      idx += entry->xyz[1] * dx;
      idx += entry->xyz[0];
      data[3] = getEndEffectorHeuristic(entry->xyz[0], entry->xyz[1], entry->xyz[2]);
      states.push_back(data);
    }
  }
}

distance_field::DistanceField* EnvironmentChain3DMoveIt::getDistanceField()
{
  return field_;
}

}  // namespace sbpl_interface
