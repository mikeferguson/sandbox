# SBPL for MoveIt

This creates a (hopefully) cleaner interface to SBPL for 7-DOF arm planning.

## Required Components
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
   * Loader utilities to create from file
 * Create distance field from robot state for evaluating cell cost - C_cell(s')
 * Run BFS for H_xyz(s). 2010 paper talks about 100^3 grid for BFS taking 0.4s to compute.
 * Add support for joint-space goals?

## Other Notes

Cost:

    C(s,s') = C_cell(s') + W_action * C_action(s,s') + W_smooth * C_smooth(s,s')

Heuristic:

    H(s) = H_xyz(s) + w*H_rpy(s)

