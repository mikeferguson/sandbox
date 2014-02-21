#!/usr/bin/env python

import time
from etherbotix.etherbotix import *

if __name__ == "__main__":
    e = EtherBotiX()
    e.send(0, [0, 0x60])
    time.sleep(.1)
    e.recv()

    print e.system_time
    print e.lm_encoder, e.rm_encoder
    print e.lm_velocity, e.rm_velocity
