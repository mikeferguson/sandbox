#!/usr/bin/env python

import rospy
from moveit_python import *

rospy.init_node("test")

g = MoveGroupInterface("arm", "base_link", plan_only = True)

# add a cupe
p = PlanningSceneInterface("base_link")
p.addCube("my_cube", 0.1, 0.5, 0.05, 0.5)

joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
positions = [0.674, 0.605, 0.268, -1.384, 1.659, -2.268, 1.204]

# plan to pose
# goal should be reachable from tucked pose, although difficult and slow
g.moveToJointPosition(joint_names, positions, tolerance = 0.5)
