/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <ros/ros.h>
#include <class_loader/class_loader.h>
#include <pluginlib/class_list_macros.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>

#include <sbpl_interface/sbpl_planning_context.h>

namespace sbpl_interface
{

SBPLPlanningContext::SBPLPlanningContext(const std::string& name, const std::string& group) :
  PlanningContext(name, group),
  sbpl_viz_(NULL)
{
}

SBPLPlanningContext::~SBPLPlanningContext()
{
}

bool SBPLPlanningContext::init(const robot_model::RobotModelConstPtr& model,
                               const sbpl_interface::SBPLPlanningParams& params)
{
  if (!model)
  {
    ROS_ERROR("No Robot Model given to the SBPLPlanningContext!");
    return false;
  }

  robot_model_ = model;
  sbpl_params_ = params;

  return true;
}

bool SBPLPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  // Results
  moveit_msgs::MotionPlanResponse mres;
  std::vector<int> solution_state_ids;
  int solution_cost;

  // Request Details
  const planning_interface::MotionPlanRequest& req = getMotionPlanRequest();
  const planning_scene::PlanningSceneConstPtr& scene = getPlanningScene();

  // Update params
  sbpl_interface::SBPLPlanningParams local_params(sbpl_params_);
  if (req.allowed_planning_time > 0.0)
    local_params.planner_params.max_time = req.allowed_planning_time;

  // Setup environment
  ros::WallTime wt = ros::WallTime::now();
  boost::shared_ptr<EnvironmentChain3DMoveIt> env_chain(new EnvironmentChain3DMoveIt);
  if (!env_chain->setupForMotionPlan(scene,
                                     req,
                                     mres,
                                     local_params))
  {
    ROS_ERROR("Unable to setup environment chain.");
    return false;
  }
  boost::this_thread::interruption_point();

  // Create planner
  boost::shared_ptr<SBPLPlanner> planner;
  if (req.planner_id.empty() || req.planner_id == "ARA*")
  {
    planner.reset(new ARAPlanner(env_chain.get(), true));
  }
  else if (req.planner_id == "AnytimeD*")
  {
    planner.reset(new ADPlanner(env_chain.get(), true));
  }
  /* These don't seem to work
  else if (req.planner_id == "ANA*")
  {
    planner.reset(new anaPlanner(env_chain.get(), true));
  }
  else if (req.planner_id == "R*")
  {
    planner.reset(new RSTARPlanner(env_chain.get(), true));
  }*/
  else
  {
    // Default is ARA*
    ROS_WARN("Unrecognized planner %s, using ARA*", req.planner_id.c_str());
    planner.reset(new ARAPlanner(env_chain.get(), true));
  }

  // Setup planner
  planner->force_planning_from_scratch();
  planner->set_start(env_chain->getStartID());
  planner->set_goal(env_chain->getGoalID());

  // Plan
  ros::WallTime plan_wt = ros::WallTime::now();
  bool b_ret = planner->replan(&solution_state_ids, local_params.planner_params, &solution_cost);
  double el = (ros::WallTime::now()-plan_wt).toSec();

  // Print stats
  ROS_INFO_STREAM("planner->replan: " << b_ret << ", planning time: " << el);

  // Do markers
  if (sbpl_viz_)
  {
    sbpl_viz_->publishExpandedStates(env_chain.get());
    sbpl_viz_->publishDistanceField(env_chain.get());
  }

  if (!b_ret)
  {
    ROS_ERROR("Planning Failed");
    env_chain->getPlanningStatistics().print();
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }

  if (solution_state_ids.size() == 0)
  {
    ROS_ERROR("Success but no path?");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  trajectory_msgs::JointTrajectory traj;
  if (!env_chain->populateTrajectoryFromStateIDSequence(solution_state_ids, mres.trajectory.joint_trajectory))
  {
    ROS_ERROR("Success but path bad");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // Try shortcutting the path
  if (sbpl_params_.attempt_shortcut)
  {
    trajectory_msgs::JointTrajectory traj = mres.trajectory.joint_trajectory;
    ROS_INFO("Planned Path is %d points", static_cast<int>(traj.points.size()));
    env_chain->attemptShortcut(traj, mres.trajectory.joint_trajectory);
  }
  ROS_INFO("Path is %d points", static_cast<int>(mres.trajectory.joint_trajectory.points.size()));

  last_planning_statistics_ = env_chain->getPlanningStatistics();
  last_planning_statistics_.total_planning_time_ = ros::WallDuration(el);
  last_planning_statistics_.total_time_ = ros::WallTime::now() - wt;
  last_planning_statistics_.print();

  // Convert  moveit_msgs::MotionPlan::Response to planning_interface::MotionPlanResponse
  res.trajectory_.reset(new robot_trajectory::RobotTrajectory(robot_model_, req.group_name));
  robot_state::RobotState start_state(robot_model_);
  res.trajectory_->setRobotTrajectoryMsg(start_state, mres.trajectory);
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  res.planning_time_ = ros::WallDuration(el).toSec();
  return true;
}

bool SBPLPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  ROS_WARN("Motion Plan Detailed Response not supported. Delegating to regular response");

  planning_interface::MotionPlanResponse undetailed_response;
  bool result = solve(undetailed_response);

  res.trajectory_.push_back(undetailed_response.trajectory_);

  std::stringstream ss;
  ss << "Trajectory for joint group " << getGroupName();
  res.description_.push_back(ss.str());

  res.processing_time_.push_back(undetailed_response.planning_time_);

  res.error_code_ = undetailed_response.error_code_;
  return result;
}

bool SBPLPlanningContext::terminate()
{
  ROS_WARN("SBPLPlanningContext::terminate unimplemented");
  return false;
}

void SBPLPlanningContext::clear()
{
  robot_model_.reset();
}

void SBPLPlanningContext::setVisualizer(SBPLVisualizerROS* viz)
{
  sbpl_viz_ = viz;
}

}  // namespace sbpl_interace
