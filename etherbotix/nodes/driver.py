#!/usr/bin/env python

"""
  EtherBotiX Node: ethernet connection to an EtherBotiX board
  Copyright (c) 2012-2014 Michael E. Ferguson.  All right reserved.

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

from etherbotix.etherbotix import *
from etherbotix.base_controller import *
from etherbotix.gps_publisher import *

class EtherbotixNode:

    def __init__(self):

        # load config
        self.rate = float(rospy.get_param('~update_rate',100.0))
        self.board = EtherBotiX()

        # base control
        self.base = BaseController(self.board)

        # gps publishing in the background?
        if rospy.get_param('~publish_gps_strings', False):
            self.gps = GPS()
            self.gps.start()

        rospy.loginfo("Done initializing.")

    def run(self):
        r = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            self.board.send(0, [0,0x40])

            if self.board.recv() > 0:
                # update base (and publish odometry)
                self.base.update_odometry()

            self.base.update()

            r.sleep()

if __name__ == '__main__':
    rospy.init_node('etherbotix')
    e = EtherbotixNode()
    e.run()

