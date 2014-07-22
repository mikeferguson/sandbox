#!/usr/bin/env python

import rospy
from moveit_python import *
from geometry_msgs.msg import *

rospy.init_node("sbpl_pose_based_goal")

g = MoveGroupInterface("arm", "base_link", plan_only = True)

# add a cube
p = PlanningSceneInterface("base_link")
p.addCube("my_cube", 0.1, 0.5, 0.05, 0.5)

# plan to pose
# goal should be reachable from tucked pose
p = PoseStamped()
p.header.frame_id = "base_link"
p.pose.position.x = 0.6
p.pose.position.y = 0.0
p.pose.position.z = 0.654
p.pose.orientation.x = 0
p.pose.orientation.y = 0
p.pose.orientation.z = 0
p.pose.orientation.w = 1.0
g.moveToPose(p, "wrist_roll_link")
