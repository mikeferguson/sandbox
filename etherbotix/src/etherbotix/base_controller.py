#!/usr/bin/env python

"""
  diff_controller.py - controller for a differential drive
  Copyright (c) 2010-2014 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its
        contributors may be used to endorse or promote products derived
        from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
from math import sin,cos,pi

from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
#from diagnostic_msgs.msg import *

from tf.broadcaster import TransformBroadcaster

class BaseController():
    """ Controller to handle movement & odometry feedback for a differential
            drive mobile base. """
    def __init__(self, device):
        self.device = device

        # the last time a command was received
        self.last_cmd_vel_time = rospy.Time.now()
        self.timeout = rospy.get_param('~cmd_vel_timeout', 0.25)

        # base parameters: wheels are 4.7" = 0.3748m * pi, 64*100 cpr
        self.ticks_meter = float(rospy.get_param('~ticks_meter', 17073))
        self.base_width = float(rospy.get_param('~base_width', 0.109375)) # TODO: this number doesn't make physical sense...
        self.accel_limit = 5

        # links for odometry publication
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')

        # internal data
        self.left_last_cmd = 0          # last command sent
        self.right_last_cmd = 0
        self.left_setpoint = 0          # cmd_vel setpoint velocity
        self.right_setpoint = 0
        self.left_last_encoder = None   # last encoder readings
        self.right_last_encoder = None
        self.last_systime = None        # last board time
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0
        self.then = rospy.Time.now()    # time for determining dx/dy

        # ROS interface
        rospy.Subscriber('base_controller/command', Twist, self.cmd_vel_cb)
        self.odomPub = rospy.Publisher("odom",Odometry)
        self.odomBroadcaster = None
        if rospy.get_param('~publish_tf', True):
            self.odomBroadcaster = TransformBroadcaster()

    def cmd_vel_cb(self, msg):
        """ Handle movement requests. """
        self.last_cmd_vel_time = rospy.Time.now()
        self.left_setpoint = -int( ((msg.linear.x - (msg.angular.z * self.base_width/2.0)) * self.ticks_meter) / 100.0)
        self.right_setpoint = int( ((msg.linear.x + (msg.angular.z * self.base_width/2.0)) * self.ticks_meter) / 100.0)

    def update_odometry(self):
        """ Update odometry using the left and right encoder values. """
        # timing
        now = rospy.Time.now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.to_sec()

        # calculate distance traveled
        if self.last_systime == None or self.device.system_time < self.last_systime:
            d_left = 0
            d_right = 0
        else:
            d_left = (self.device.lm_encoder - self.left_last_encoder)/self.ticks_meter
            d_right = (self.device.rm_encoder - self.right_last_encoder)/self.ticks_meter
        self.left_last_encoder = self.device.lm_encoder
        self.right_last_encoder = self.device.rm_encoder
        self.last_systime = self.device.system_time

        d = (d_left+d_right)/2
        th = (d_right-d_left)/self.base_width

        if (d != 0):
            x = cos(th)*d
            y = -sin(th)*d
            self.x = self.x + (cos(self.th)*x - sin(self.th)*y)
            self.y = self.y + (sin(self.th)*x + cos(self.th)*y)
        if (th != 0):
            self.th = self.th + th

        # calculate base velocity
        dx = (self.device.lm_velocity+self.device.rm_velocity)/(2*self.ticks_meter) * 100.
        dr = (self.device.rm_velocity-self.device.lm_velocity)/(self.ticks_meter)/self.base_width * 100

        # publish or perish?
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th/2)
        quaternion.w = cos(self.th/2)
        if self.odomBroadcaster:
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
                )

        odom = Odometry()
        odom.header.frame_id = self.odom_frame_id
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = dx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = dr
        self.odomPub.publish(odom)

    def update(self):
        """ Send updated velocities to base.  """
        now = rospy.Time.now()
        if now > (self.last_cmd_vel_time + rospy.Duration(self.timeout)):
            self.left_setpoint = 0
            self.right_setpoint = 0

        if self.left_last_cmd < self.left_setpoint:
            self.left_last_cmd += self.accel_limit
            if self.left_last_cmd > self.left_setpoint:
                self.left_last_cmd = self.left_setpoint
        else:
            self.left_last_cmd -= self.accel_limit
            if self.left_last_cmd < self.left_setpoint:
                self.left_last_cmd = self.left_setpoint

        if self.right_last_cmd < self.right_setpoint:
            self.right_last_cmd += self.accel_limit
            if self.right_last_cmd > self.right_setpoint:
                self.right_last_cmd = self.right_setpoint
        else:
            self.right_last_cmd -= self.accel_limit
            if self.right_last_cmd < self.right_setpoint:
                self.right_last_cmd = self.right_setpoint

        self.device.send(self.device.ADDR_LM_VELOCITY, self.device.write_2(self.left_last_cmd) + self.device.write_2(self.right_last_cmd))

