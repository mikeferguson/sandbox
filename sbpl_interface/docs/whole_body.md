## Notes on whole body planning issues

MoveIt allows planning for non-single-dof joints, however the interface is a bit
different than that for single-dof joints.

Notes:
 * moveit::core::JointModel
   * Has variables, with names. For single DOF joints, this is just joint name.
     For multi-DOF joints, this is joint name plus "/" plus the variable (x, theta, etc)
   * getVariableCount() -- 0 is fixed, 1 is single DOF, all others are multi-DOF
 * moveit::core::JointModelGroup
   * isSingleDOFJoints() returns if any joints in the group are not single DOF.
 * moveit::core::RobotState
   * setJointPositions(std::string name, double position) can be used for single DOF joints
   * setJointPositions(std::string name, Eigen::Affine3d transform) can be used
     for multi-DOF joints. The name is the joint name, not including the variables.

Other Issues:
 * MotionPrimitives currently do degree->radian conversion, this will throw off any
   mobile base primitives.
 * DistanceField will need to grow and/or move?
