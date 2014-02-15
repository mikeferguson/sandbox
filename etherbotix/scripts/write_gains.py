#!/usr/bin/env python

import sys
from etherbotix.etherbotix import *

def set_gain(name, gain, shift, eth):
    if name == 'lkp':
        eth.send(eth.ADDR_LM_KP_GAIN,[gain, shift])
    elif name == 'lki':
        eth.send(eth.ADDR_LM_KI_GAIN,[gain, shift])
    elif name == 'lkd':
        eth.send(eth.ADDR_LM_KD_GAIN,[gain, shift])
    elif name == 'rkp':
        eth.send(eth.ADDR_RM_KP_GAIN,[gain, shift])
    elif name == 'rki':
        eth.send(eth.ADDR_RM_KI_GAIN,[gain, shift])
    elif name == 'rkd':
        eth.send(eth.ADDR_RM_KD_GAIN,[gain, shift])
    else:
        return 0
    return 1

    e = EtherBotiX()
    e.send(e.ADDR_LM_VELOCITY, e.write_2(int(sys.argv[1])) + e.write_2(int(sys.argv[2])))

def set_windup(name, windup, eth):
    if name == 'lw':
        eth.send(eth.ADDR_LM_WINDUP,[windup])
    elif name == 'rw':
        eth.send(eth.ADDR_RM_WINDUP,[windup])
    else:
        return 0
    return 1

def run():
    e = EtherBotiX()
    if len(sys.argv) > 2:
        if len(sys.argv) > 3:
            shift = int(sys.argv[3])
            if shift < 0:
                shift = -shift + 0x80
            return set_gain(sys.argv[1], int(sys.argv[2]), shift, e)
        #else:
        #    return set_windup(sys.argv[1], int(sys.argv[2]), e)
    else:
        return 0

if __name__=="__main__":
    if run() == 0:
        print "usage: write_gains.py gain_name gain shift"
        
