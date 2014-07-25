# SBPL for MoveIt

This creates a (hopefully) cleaner interface to SBPL for n-DOF arm planning.

## Overall status:
 * Supports joint and pose constrained goals
 * Should support path constraints, but is untested
 * Lots of parameters, described in sbpl_planning_params.h. See also
   [ubr1](https://github.com/mikeferguson/ubr1_preview/blob/sbpl/ubr1_moveit/config/sbpl_planning.yaml)
   config for examples.
 * Works with pick/place -- but only single threaded
 * Collision checking is slow -- planning takes about 3x longer than the groovy/sbpl_arm_planner

## TODO (in order of expected/anticipated severeness)
 * Distance field is recreated each time env_chain3d_moveit.setupForMotionPlan is called (wasteful)
 * BUG: Need to implement terminate to work with multithreaded pick/place
 * BUG: Something isn't thread happy -- causes assertion in pick/place. Appears to be related to BFS thread?
   However this did not occur before with BFS? Maybe something in the motion primitives?

        move_group: /usr/include/boost/thread/pthread/recursive_mutex.hpp:101:
        boost::recursive_mutex::~recursive_mutex(): Assertion `!pthread_mutex_destroy(&m)' failed

 * BUG: Distance field computations appear not to fill all occupied cells for a box?
 * ENHANCEMENT: use MotionPlanRequest/workspace_parameters to define BFS/distance field size, overriding sbpl_params.
 * ENHANCEMENT: add snap_to_rpy (orientation solver from sbpl_arm_planner)

## Components
 * Env3dHashData - the actual graph data structure
 * BFS - the heuristic data structure
 * EnvironmentChain3d (plus statistics and parameters)
   * Needs access to planning_scene, robot_state
   * Create distance field from robot state for evaluating cell cost - C_cell(s')
   * Run BFS for H_xyz(s). 2010 paper talks about 100^3 grid for BFS taking 0.4s to compute.
 * Motion Primitives
   * Simple n-tuples for n joints
     * sbpl_motion_planning seems to use same prims for both short and long distance
     * long distance only uses the upper 4 joints of the arm
     * short distance might be good with 4 joints in more open spaces, needs all joints in more cluttered space.
   * Snap to XYZRPY (do ik when within predefined distance of goal)
     * The distance to goal is computed from the BFS distance so that it includes obstacles.
     * Distance threshold is specified as 10cm in ICRA2011 paper.
   * Snap to RPY (orientation solver)
     * This doesn't appear to be used at first look in sbpl_motion_planning, however,
       it's baked into the computeIK call.
   * Motion Primitives are mostly accessed through the loader utilities to create
     primitives from file
     * While not particularly clean, the complex primitives (the IK solver and snap
       to joint) are currently created in EnvironmentChain3DMoveIt::setupForMotionPlan,
       because the state data is available there to create them.

## Other Notes

Cost as defined in the paper is:

    C(s,s') = C_cell(s') + W_action * C_action(s,s') + W_smooth * C_smooth(s,s')

Currently, this cost is simply 1 for all actions and all smoothness... :(

Heuristic:

    H(s) = H_xyz(s) + w*H_rpy(s)

ARAPlanner implementation has the following fields (and others):

    struct ARASEARCHSTATEDATA
    {
      MDPState * state;
      unsigned int v;   // best cost to get from start to this state -- set equal to g during expansion of the state
      unsigned int g;   // best cost to get from start to this state -- assigned during UpdateSucc
      int h;            // this is the heuristic value -- gets assigned as state is initialized
      MDPState * bestnextstate;
    };

During expansion:

    UpdateSucc(state, space)
    {
      env->GetSuccs(state, [succ], [cost])
      for (succ)
      {
        cost = cost of succ
        if (new state)
          ReinitializeSearchStateInfo(succ)
          // sets g = v = INF
          // sets h = GetGoalHeuristic(succ)
        if (succ->g > state->v + cost)
          succ->g = state->v + cost
          succ->bestpredstate = state
          if (succ is in open set)
            add succ to heap with cost (g + eps * h)
      }
    }

Thus, costs come only through GetSuccs, and heuristic comes only through GetGoalHeuristic
(or GetStartHeuristic if doing backwards planning, but we are not).

The costs used in various portions of the planners appear to be mostly magical,
and often hard coded. The most up to date planner is the sbpl_manipulation package
on Github, but there are also older versions on the sbpl.net site. All costs are
integers. A summary of costs follows:

 * sbpl_manipulation
    * prm_->cost_per_cell_ = 100 (initialized to 1 in planning params,
      but computeCostPerCell() is called in initEnvironment, setting it to 100)
    * prm_->cost_multiplier_ = 1000
    * prm_->cost_per_meter_ = 50
    * Available Heuristics -- used to get h(s)
      * bfs_cost_to_goal = distance * prm_->cost_per_cell_
      * euclidean_cost_to_goal = distance_in_meters * prm_->cost_per_meter_
    * GetSuccs calls the cost() function:
      * which just returns prm_->cost_multiplier_
    * There are functions defined for the action cost, but they appear unused?
      * Does not use forearm roll or wrist roll for action cost computation
      * Finds max difference between two actions
      * cost = (max_diff/prm_->max_mprim_offset_) * prm_->cost_multiplier_
 * Diamondback/Electric (arm_navigation) versions
   * Mostly the same as above.
   * cost() has choice of uniform, or cost based on distance to nearest obstacle
   * defines 4 different ranges of closeness, each with own scale:
     * dist less than 7 = cost_multiplier_ * 12
     * dist less than 12 = cost_multiplier_ * 7
     * dist less than 17 = cost_multiplier_ * 2
     * all others = cost_multiplier_
 * Fuerte/MoveIt version
   * Mostly the same as Diamondback/Electric
   * IK/RPY solvers return extra motion primitive cost which is added to the standard cost.
   * In each case, these extra costs use the action cost method described above.

## Future

 * Support multiple goals -- especially for pick and place?
   * Push multiple goals into MotionPlanRequest goal_contraints array (message is defined that way).
   * Update isGoalState to handle the array.
   * Update BFS to handle multiple start points.
 * Support collision checking in distance field (see MoveIt_experimental CollisionWorldHybrid,
   https://github.com/ros-planning/moveit_core/commit/e1cb349ecfd2dad8b61d1b0d3717036175ce61ba)
