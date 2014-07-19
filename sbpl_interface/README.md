# SBPL for MoveIt

This creates a (hopefully) cleaner interface to SBPL for 7-DOF arm planning.

## Status/TODO (in order of expected/anticipated severeness)
 * Look into cost function (env3d::calculateCost)
 * Load motion primitives from parameters, with decent defaults if no parameter exists. (sbpl_planner_params.h)
 * There is no smoothness cost assigned to motion primitive transitions.
 * Lacks visualization:
   * Need a publisher for BFS and/or distance field
   * Visualization of expanded states?
 * Snap to XYZRPY, Snap to RPY are not implemented (there is snap_to_joint, which works for joint-space requests)
 * Distance field is recreated each time env_chain3d_moveit.setupForMotionPlan is called (wasteful)
 * BUG: Goal state retains angles from first assignment -- this will be a problem when using pose constraints rather than joint constraints

## Required Components
 * PlanningData - the actual graph data structure
 * BFS - the heuristic data structure
 * Environment (plus statistics and parameters)
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
   * Loader utilities to create primitives from file

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

## Future

 * Support multiple goals -- especially for pick and place?
 * Support collision checking in distance field (see MoveIt_experimental CollisionWorldHybrid,
   https://github.com/ros-planning/moveit_core/commit/e1cb349ecfd2dad8b61d1b0d3717036175ce61ba)
