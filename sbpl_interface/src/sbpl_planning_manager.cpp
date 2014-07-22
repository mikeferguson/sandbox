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

#include <class_loader/class_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <sbpl_interface/sbpl_planning_context.h>
#include <sbpl_interface/sbpl_planning_params.h>
#include <sbpl_interface/sbpl_visualizer_ros.h>

namespace sbpl_interface
{

class SBPLPlannerManager : public planning_interface::PlannerManager
{
public:
  SBPLPlannerManager() : planning_interface::PlannerManager(),
                         nh_("~"),
                         sbpl_viz_(NULL)
  {
  }

  ~SBPLPlannerManager()
  {
    if (sbpl_viz_)
      delete sbpl_viz_;
  }

  virtual bool initialize(const robot_model::RobotModelConstPtr& model, const std::string &ns)
  {
    if (!ns.empty())
      nh_ = ros::NodeHandle(ns);

    ROS_INFO("Initializing SBPL Planner Manager with robot \"%s\" in namespace \"%s\"", model->getName().c_str(), ns.c_str());

    // Hold onto robot model
    if (!model)
    {
      ROS_ERROR("Planner Manager received Null Robot Model");
      return false;
    }
    robot_model_ = model;

    // Load parameters from param server
    sbpl_params_ = new sbpl_interface::SBPLPlanningParams();
    if (!(sbpl_params_ && sbpl_params_->init(nh_)))
    {
      ROS_ERROR("Cannot init SBPLPlanningParams");
      return false;
    }
    sbpl_params_->print();

    // Setup visualization (handles its own parameters)
    sbpl_viz_ = new SBPLVisualizerROS(nh_);

    return true;
  }

  virtual bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
  {
    return req.trajectory_constraints.constraints.empty();
  }

  virtual std::string getDescription() const
  {
    return "SBPL";
  }

  virtual void getPlanningAlgorithms(std::vector<std::string> &algs) const
  {
    algs.clear();
    algs.push_back("ARA*");
    algs.push_back("AnytimeD*");
    //algs.push_back("ANA*");  TODO: these aren't working, see context for more
    //algs.push_back("R*");
  }

  virtual void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pconfig)
  {
    // TODO
  }

  virtual planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr& planning_scene,
      const planning_interface::MotionPlanRequest &req,
      moveit_msgs::MoveItErrorCodes &error_code) const
  {
    sbpl_interface::SBPLPlanningContext* sbpl_planning_context =
      new sbpl_interface::SBPLPlanningContext(req.group_name, req.group_name);

    planning_interface::PlanningContextPtr fresh_context;

    if (!sbpl_planning_context)
    {
      ROS_ERROR("Failed to instanciate SBPL Manipulation Planning Context");
      error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE; 
    }
    else
    {
      // TODO: if req has orientation constraints and params include orientation sovler, add it
      // TODO: if req has position and orientation constraints and params include IK solver, add it
      SBPLPlanningParams params(*sbpl_params_);

      sbpl_planning_context->setMotionPlanRequest(req);
      sbpl_planning_context->setPlanningScene(planning_scene);
      sbpl_planning_context->setVisualizer(sbpl_viz_);

      if (!sbpl_planning_context->init(robot_model_, params))
      {
        ROS_ERROR("Failed to initialize SBPL Manipulation Planning Context");
        error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE; 
      }
      else
      {
        fresh_context.reset(sbpl_planning_context);
        error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      }
    }

    return fresh_context;
  }
  
  void terminate() const
  {
    // TODO
  }

private:
  ros::NodeHandle nh_;
  robot_model::RobotModelConstPtr robot_model_;
  sbpl_interface::SBPLPlanningParams* sbpl_params_;
  sbpl_interface::SBPLVisualizerROS* sbpl_viz_;
};

}  // namespace sbpl_interface

CLASS_LOADER_REGISTER_CLASS(sbpl_interface::SBPLPlannerManager, planning_interface::PlannerManager);
