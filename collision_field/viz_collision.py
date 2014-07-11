#!/usr/bin/env python

##
## Visualization of https://github.com/bcohen/sbpl_manipulation/commit/81642dc3e6142940c9f91dda772ebf9f3af5ebfa
##

import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray

def createSphereMarker(d, r = 0.0, g = 0.8, b = 0.0, a = 1):
    m = Marker()
    # now, in our frame, with ns = our name
    m.header.stamp = rospy.Time.now()
    m.header.frame_id = d["link"]
    m.ns = d["name"]
    # add a sphere
    m.type = m.SPHERE
    m.action = m.ADD
    # create pose
    m.pose = Pose()
    m.pose.position.x = d["x"]
    m.pose.position.y = d["y"]
    m.pose.position.z = d["z"]
    m.pose.orientation.w = 1.0
    # xyz
    m.scale.x = d["radius"]*2
    m.scale.y = d["radius"]*2
    m.scale.z = d["radius"]*2
    m.color.r = r
    m.color.g = g
    m.color.b = b
    m.color.a = a
    return m

spheres = list()

spheres.append({"name": "sh0", "x": 0.14, "y": -0.06, "z": -0.01, "radius": 0.095, "link": "shoulder_pan_link"})
spheres.append({"name": "sh1", "x": 0.14, "y": 0.06, "z": -0.01, "radius": 0.095, "link": "shoulder_pan_link"})
spheres.append({"name": "sh2", "x": 0.14, "y": 0.00, "z": -0.01, "radius": 0.095, "link": "shoulder_pan_link"})

spheres.append({"name": "ua0", "x": 0.02, "y": 0.0, "z": 0.0, "radius": 0.09, "link": "upperarm_roll_link"})
spheres.append({"name": "ua1", "x": 0.06, "y": 0.03, "z": -0.0225, "radius": 0.06, "link": "upperarm_roll_link"})
spheres.append({"name": "ua2", "x": 0.06, "y": -0.03, "z": -0.0225, "radius": 0.06, "link": "upperarm_roll_link"})
spheres.append({"name": "ua9", "x": 0.06, "y": -0.0, "z": -0.0225, "radius": 0.06, "link": "upperarm_roll_link"})

spheres.append({"name": "ua7", "x": 0.11, "y": -0.04, "z": -0.035, "radius": 0.05, "link": "upperarm_roll_link"})
spheres.append({"name": "ua8", "x": 0.11, "y": 0.04, "z": -0.035, "radius": 0.05, "link": "upperarm_roll_link"})
spheres.append({"name": "ua10", "x": 0.11, "y": 0.0, "z": -0.035, "radius": 0.05, "link": "upperarm_roll_link"})

spheres.append({"name": "ua3", "x": 0.16, "y": 0.03, "z": -0.025, "radius": 0.06, "link": "upperarm_roll_link"})
spheres.append({"name": "ua4", "x": 0.16, "y": -0.03, "z": -0.025, "radius": 0.06, "link": "upperarm_roll_link"})
spheres.append({"name": "ua11", "x": 0.16, "y": -0.0, "z": -0.025, "radius": 0.06, "link": "upperarm_roll_link"})

spheres.append({"name": "ua5", "x": 0.21, "y": 0.035, "z": 0.0, "radius": 0.06, "link": "upperarm_roll_link"})
spheres.append({"name": "ua6", "x": 0.21, "y": -0.035, "z": 0.0, "radius": 0.06, "link": "upperarm_roll_link"})

spheres.append({"name": "fa1", "x": 0.05, "y": 0.025, "z": -0.005, "radius": 0.065, "link": "forearm_roll_link"})
spheres.append({"name": "fa2", "x": 0.05, "y": -0.025, "z": -0.005, "radius": 0.065, "link": "forearm_roll_link"})
spheres.append({"name": "fa3", "x": 0.10, "y": 0.0275, "z": -0.005, "radius": 0.065, "link": "forearm_roll_link"})
spheres.append({"name": "fa4", "x": 0.10, "y": -0.0275, "z": -0.005, "radius": 0.065, "link": "forearm_roll_link"})
spheres.append({"name": "fa5", "x": 0.15, "y": 0.0225, "z": -0.015, "radius": 0.05, "link": "forearm_roll_link"})
spheres.append({"name": "fa6", "x": 0.15, "y": -0.0225, "z": -0.015, "radius": 0.05, "link": "forearm_roll_link"})
spheres.append({"name": "fa7", "x": 0.19, "y": 0.03, "z": -0.015, "radius": 0.05, "link": "forearm_roll_link"})
spheres.append({"name": "fa8", "x": 0.19, "y": -0.03, "z": -0.015, "radius": 0.05, "link": "forearm_roll_link"})
spheres.append({"name": "fa9", "x": 0.24, "y": 0.03, "z": 0.00, "radius": 0.05, "link": "forearm_roll_link"})
spheres.append({"name": "fa10", "x": 0.24, "y": -0.03, "z": 0.0, "radius": 0.05, "link": "forearm_roll_link"})

spheres.append({"name": "gr0", "x": 0.055, "y": -0.03, "z": 0.00, "radius": 0.04, "link": "wrist_roll_link"})
spheres.append({"name": "gr1", "x": 0.055, "y": 0.0, "z": 0.00, "radius": 0.04, "link": "wrist_roll_link"})
spheres.append({"name": "gr2", "x": 0.055, "y": 0.03, "z": 0.00, "radius": 0.04, "link": "wrist_roll_link"})
spheres.append({"name": "gr3", "x": 0.025, "y": 0.02, "z": 0.00, "radius": 0.04, "link": "wrist_roll_link"})
spheres.append({"name": "gr4", "x": 0.025, "y": -0.02, "z": 0.00, "radius": 0.04, "link": "wrist_roll_link"})

spheres.append({"name": "lf0", "x": -0.02, "y": 0.005, "z": 0.00, "radius": 0.015, "link": "left_gripper_finger_link"})
spheres.append({"name": "lf1", "x": -0.01, "y": 0.005, "z": 0.00, "radius": 0.015, "link": "left_gripper_finger_link"})
spheres.append({"name": "lf2", "x": 0.00, "y": 0.005, "z": 0.00, "radius": 0.015, "link": "left_gripper_finger_link"})
spheres.append({"name": "lf3", "x": 0.01, "y": 0.005, "z": 0.00, "radius": 0.015, "link": "left_gripper_finger_link"})
spheres.append({"name": "lf4", "x": 0.02, "y": 0.005, "z": 0.00, "radius": 0.015, "link": "left_gripper_finger_link"})

spheres.append({"name": "rf0", "x": -0.02, "y": -0.005, "z": 0.00, "radius": 0.015, "link": "right_gripper_finger_link"})
spheres.append({"name": "rf1", "x": -0.01,   "y": -0.005, "z": 0.00, "radius": 0.015, "link": "right_gripper_finger_link"})
spheres.append({"name": "rf2", "x": 0.0, "y": -0.005, "z": 0.00, "radius": 0.015, "link": "right_gripper_finger_link"})
spheres.append({"name": "rf3", "x": 0.01, "y": -0.005, "z": 0.00, "radius": 0.015, "link": "right_gripper_finger_link"})
spheres.append({"name": "rf4", "x": 0.02, "y": -0.005, "z": 0.00, "radius": 0.015, "link": "right_gripper_finger_link"})

rospy.init_node("viz_collision")
pub = rospy.Publisher("collision", MarkerArray, queue_size=10, latch=True)

msg = MarkerArray()
for s in spheres:
    msg.markers.append(createSphereMarker(s))

pub.publish(msg)
rospy.spin()


