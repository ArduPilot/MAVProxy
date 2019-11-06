#!/usr/bin/env python
'''Decode RTCM v3 messages'''

RTCMv3_PREAMBLE = 0xD3
POLYCRC24 = 0x1864CFB

import struct

class RTCM3:
    def __init__(self, debug=False):
        self.crc_table = None
        self.debug = debug
        self.reset()

    def get_packet(self):
        '''return bytearray of last parsed packet'''
        return self.parsed_pkt

    def get_packet_ID(self):
        '''get get of packet, or None'''
        if self.parsed_pkt is None or len(self.parsed_pkt) < 8:
            return None
        id, = struct.unpack('>H', self.parsed_pkt[3:5])
        id >>= 4
        return id

    def reset(self):
        '''reset state'''
        self.pkt = bytearray()
        self.pkt_len = 0
        self.parsed_pkt = None

    def parse(self):
        '''parse packet'''
        parity = self.pkt[-3:]
        crc1 = parity[0] << 16 | parity[1] << 8 | parity[2]
        crc2 = self.crc24(self.pkt[:-3])
        if crc1 != crc2:
            if self.debug:
                print("crc fail len=%u" % len(self.pkt))
            # look for preamble
            idx = self.pkt[1:].find(bytearray([RTCMv3_PREAMBLE]))
            if idx >= 0:
                self.pkt = self.pkt[1+idx:]
                if len(self.pkt) >= 3:
                    self.pkt_len, = struct.unpack('>H', self.pkt[1:3])
                    self.pkt_len &= 0x3ff
                else:
                    self.pkt_len = 0
            else:
                self.reset()
            return False

        # got a good packet
        self.parsed_pkt = self.pkt
        self.pkt = bytearray()
        self.pkt_len = 0
        return True

    def read(self, byte):
        '''read in one byte, return true if a full packet is available'''

        #import random
        #if random.uniform(0,1000) < 1:
        #    return False

        byte = ord(byte)
        if len(self.pkt) == 0 and byte != RTCMv3_PREAMBLE:
            # discard
            return False

        self.pkt.append(byte)
        if self.pkt_len == 0 and len(self.pkt) >= 3:
            self.pkt_len, = struct.unpack('>H', self.pkt[1:3])
            self.pkt_len &= 0x3ff
            if self.pkt_len == 0:
                self.reset()
                return False

        if self.pkt_len > 0 and len(self.pkt) >= 3 + self.pkt_len + 3:
            remainder = self.pkt[6+self.pkt_len:]
            self.pkt = self.pkt[:6+self.pkt_len]
            # got header, packet body and parity
            ret = self.parse()
            self.pkt.extend(remainder)
            return ret

        # need more bytes
        return False

    def crc24(self, bytes):
        '''calculate 24 bit crc'''
        if self.crc_table is None:
            # initialise table
            self.crc_table = [0] * 256
            for i in range(256):
                self.crc_table[i] = i<<16
                for j in range(8):
                    self.crc_table[i] <<= 1
                    if (self.crc_table[i] & 0x1000000):
                        self.crc_table[i] ^= POLYCRC24
        crc = 0
        for b in bytes:
            crc = ((crc<<8)&0xFFFFFF) ^ self.crc_table[(crc>>16) ^ b]
        return crc

if __name__ == '__main__':
    from argparse import ArgumentParser
    import time
    parser = ArgumentParser(description='RTCM3 parser')

    parser.add_argument("filename", type=str, help="input file")
    parser.add_argument("--debug", action='store_true', help="show errors")
    parser.add_argument("--follow", action='store_true', help="continue reading on EOF")
    args = parser.parse_args()

    rtcm3 = RTCM3(args.debug)
    f = open(args.filename, 'rb')
    while True:
        b = f.read(1)
        if len(b) == 0:
            if args.follow:
                time.sleep(0.1)
                continue
            break
        if rtcm3.read(b):
            print("packet len %u ID %u" % (len(rtcm3.get_packet()), rtcm3.get_packet_ID()))
