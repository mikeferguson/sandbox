#!/usr/bin/env python

import socket
import time

class EtherBotiX:

    ADDR_MODEL_NUMBER = 0x02
    ADDR_VERSION = 0x04

    ADDR_SYSTEM_TIME = 0x20
    ADDR_SYSTEM_VOLTAGE = 0x24
    ADDR_SYSTEM_FLAGS = 0x26
    ADDR_AUX_CURRENT = 0x28
    ADDR_PKTS_RECV = 0x20

    ADDR_LM_ENCODER = 0x30
    ADDR_RM_ENCODER = 0x34
    ADDR_LM_VELOCITY = 0x38
    ADDR_RM_VELOCITY = 0x3A
    ADDR_LM_CURRENT = 0x3C
    ADDR_RM_CURRENT = 0x3E

    ADDR_ACC_X = 0x40
    ADDR_ACC_Y = 0x42
    ADDR_ACC_Z = 0x44
    ADDR_GYRO_X = 0x46
    ADDR_GYRO_Y = 0x48
    ADDR_GYRO_Z = 0x4A
    ADDR_MAG_X = 0x4C
    ADDR_MAG_Y = 0x4E

    ADDR_MAG_Z = 0x50

    ADDR_LM_KP_GAIN = 0x52
    ADDR_LM_KP_SHIFT = 0x53
    ADDR_LM_KI_GAIN = 0x54
    ADDR_LM_KI_SHIFT = 0x55
    ADDR_LM_KD_GAIN = 0x56
    ADDR_LM_KD_SHIFT = 0x57

    ADDR_RM_KP_GAIN = 0x58
    ADDR_RM_KP_SHIFT = 0x59
    ADDR_RM_KI_GAIN = 0x5A
    ADDR_RM_KI_SHIFT = 0x5B
    ADDR_RM_KD_GAIN = 0x5C
    ADDR_RM_KD_SHIFT = 0x5D

    # turrible hacks
    MAGIC = chr(255) + 'BOT'
    SIGNED = True

    def __init__(self, ip="192.168.0.41", port=6707):
        self._ip = ip
        self._port = port
        self._conn = socket.socket (socket.AF_INET, socket.SOCK_DGRAM)
        self._conn.bind( ("", 0) )
        self._conn.setblocking(0)
        self.messages = dict()

        self.model = 0
        self.version = 0
        
        self.system_time = 0
        self.system_voltage = 0
        self.system_flags = 0
        self.aux_current = 0
        self.pkts_recv = 0

        self.lm_encoder = 0
        self.rm_encoder = 0
        self.lm_velocity = 0
        self.rm_velocity = 0
        self.lm_current = 0
        self.rm_current = 0

        self.acc_x = 0
        self.acc_y = 0
        self.acc_z = 0
        self.gyro_x = 0
        self.gyro_y = 0
        self.gyro_z = 0
        self.mag_x = 0
        self.mag_y = 0
        self.mag_z = 0

    ###################################
    ## Socket Functions

    def recv(self):
        """ read available packets from the EtherBotiX """
        pkts = 0
        try:
            while(True):
                values = self._conn.recv(1024)
                if values[0:4] != self.MAGIC:
                    print "failed packet: magic number is invalid"
                p_addr = ord(values[4])
                p_len = ord(values[5])
                data = [ord(d) for d in values[6:]]
                if p_len != len(data):
                    print "failed packet: length and parameter count do not match"
                self.model = self.attempt_get(self.model, self.ADDR_MODEL_NUMBER, 2, p_addr, p_len, data)
                self.version = self.attempt_get(self.version, self.ADDR_VERSION, 2, p_addr, p_len, data)

                self.system_time = self.attempt_get(self.system_time, self.ADDR_SYSTEM_TIME, 4, p_addr, p_len, data)
                self.system_voltage = self.attempt_get(self.system_voltage, self.ADDR_SYSTEM_TIME, 2, p_addr, p_len, data)
                self.system_flags = self.attempt_get(self.system_flags, self.ADDR_SYSTEM_TIME, 2, p_addr, p_len, data)

                self.lm_encoder = -self.attempt_get(self.lm_encoder, self.ADDR_LM_ENCODER, 4, p_addr, p_len, data, self.SIGNED)
                self.rm_encoder = self.attempt_get(self.rm_encoder, self.ADDR_RM_ENCODER, 4, p_addr, p_len, data, self.SIGNED)
                self.lm_velocity = -self.attempt_get(self.lm_velocity, self.ADDR_LM_VELOCITY, 2, p_addr, p_len, data, self.SIGNED)
                self.rm_velocity = self.attempt_get(self.rm_velocity, self.ADDR_RM_VELOCITY, 2, p_addr, p_len, data, self.SIGNED)

                self.acc_x = self.attempt_get(self.acc_x, self.ADDR_ACC_X, 2, p_addr, p_len, data, self.SIGNED)
                self.acc_y = self.attempt_get(self.acc_y, self.ADDR_ACC_Y, 2, p_addr, p_len, data, self.SIGNED)
                self.acc_z = self.attempt_get(self.acc_z, self.ADDR_ACC_Z, 2, p_addr, p_len, data, self.SIGNED)
                self.gyro_x = self.attempt_get(self.gyro_x, self.ADDR_GYRO_X, 2, p_addr, p_len, data, self.SIGNED)
                self.gyro_y = self.attempt_get(self.gyro_y, self.ADDR_GYRO_Y, 2, p_addr, p_len, data, self.SIGNED)
                self.gyro_z = self.attempt_get(self.gyro_z, self.ADDR_GYRO_Z, 2, p_addr, p_len, data, self.SIGNED)
                self.mag_x = self.attempt_get(self.mag_x, self.ADDR_MAG_X, 2, p_addr, p_len, data, self.SIGNED)
                self.mag_y = self.attempt_get(self.mag_y, self.ADDR_MAG_Y, 2, p_addr, p_len, data, self.SIGNED)
                self.mag_z = self.attempt_get(self.mag_z, self.ADDR_MAG_Z, 2, p_addr, p_len, data, self.SIGNED)
                pkts += 1
        except:
            pass
        return pkts

    def send(self, addr, params):
        """ send a packet to the EtherBotiX """
        msg = self.MAGIC + chr(addr) + chr(len(params)) + "".join([chr(p) for p in params])
        self._conn.sendto(msg, 0, (self._ip,self._port))

    ###################################
    ## Data Helpers

    def attempt_get(self, var, addr, len, p_addr, p_len, data, signed=False):
        """ try to read a value from data, if data will contain that address """
        if p_addr > addr or addr + len > p_addr + p_len:
            return var
        if len == 1:
            return data[addr - p_addr]
        elif len == 2:
            return self.read_2(data, addr - p_addr, signed)
        elif len == 4:
            return self.read_4(data, addr - p_addr, signed)
            
    def read_2(self, data, address, signed=False):
        d = data[address] + (data[address+1]<<8)
        if signed and d > 32768:
            return d - 65536
        return d

    def read_4(self, data, address, signed=False):
        d = data[address] + (data[address+1]<<8) + (data[address+2]<<16) + (data[address+3]<<24)
        if signed and d > 2147483648:
            return d - 4294967296
        return d

    def write_2(self, value):
        return [value&0xff, (value>>8)&0xff]
    
    def write_4(self, value):
        return [value&0xff, (value>>8)&0xff, (value>>16)&0xff, (value>>24)&0xff]
    
