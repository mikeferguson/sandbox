#!/usr/bin/env python

import socket
import time

import rospy
from nmea_msgs.msg import Sentence

class GPS:

    # turrible hacks
    MAGIC = chr(255) + 'BOT'

    def __init__(self, ip="192.168.0.41", port=6707):
        self._ip = ip
        self._port = port
        self._conn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._conn.bind( ("", 6708) ) # we need to be port 6708
        self._conn.setblocking(0)
        
        self._pub = rospy.Publisher("nmea_sentence", Sentence)

    ###################################
    ## Socket Functions

    def run(self):
        while not rospy.is_shutdown():
            raw_s = self.recv()
            if raw_s:
                s = Sentence()
                # TODO: s.header
                s.sentence = raw_s
                self._pub.publish(s)

    def recv(self):
        """ read available packets from the EtherBotiX """
        try:
            while(True):
                values = self._conn.recv(1024)
                if values[0:4] != self.MAGIC:
                    print "failed packet: magic number is invalid"
                p_addr = ord(values[4])
                p_len = ord(values[5])
                return values[6:]
        except:
            pass
        return None

    def send(self, addr, params):
        """ send a packet to the EtherBotiX """
        msg = self.MAGIC + chr(addr) + chr(len(params)) + "".join([chr(p) for p in params])
        self._conn.sendto(msg, 0, (self._ip,self._port))

