#!/usr/bin/env python

import rospy
from moveit_python import *

# start  -1.29692 1.3439 -2.9374 -1.81008 -0.107271 -1.21004 0.173851 
# start 286 77 192 256 354 291 10
# goal 342 46 235 255 32 279 355

rospy.init_node("test")
g = MoveGroupInterface("arm", "base_link", plan_only = True)
p = PlanningSceneInterface("base_link")
p.addCube("my_cube", 0.1, 0.5, 0.05, 0.5)

joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
#positions = [-0.31785468593, 0.80183544664, -2.1750170769, -1.82634607172, 0.560296586215, -1.41448883718, -0.0857886811548]
#positions = [-0.31785468593, 0.80183544664, -2.1750170769, -1.82634607172, 0.560296586215, -1.41448883718, -0.0857886811548]
positions = [0.0 for i in range(7)]

g.moveToJointPosition(joint_names, positions, tolerance = 0.5)
