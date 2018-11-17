#!/usr/bin/python

# motion-driver-client.py
# A PC application for use with Motion Driver.
# Copyright 2012 InvenSense, Inc. All Rights Reserved.

import serial, sys, time, string, pygame
from ponycube import *

# Sensor sensitivities
# Incoming Accel data is in fixed point. Where 1g = 2^16.
# ACCEL_CONVERSION = 9.80665 / 2^16 (converting g to m/s^2)
ACCEL_CONVERSION = 0.000149637603759766
# Incoming Gyro data is in fixed point. Where 1dps = 2^16.
# GYRO_CONVERSION = PI / 2^16 / 180 (converting dps to rads/s)
GYRO_CONVERSION  = 2.66316109007924e-007
# Incoming Compass data is in fixed point. Where 1uT = 2^16.
# COMPASS_CONVERSION = 1 / 2^16
COMPASS_CONVERSION = 1.52587890625e-005
# Incoming Quaternion data is in Q30 fixed point.
# QUAT_CONVERSION = 1 / 2^30
QUAT_CONVERSION  = 9.31322574615478515625e-10


class motion_driver_packet_reader:
    def __init__(self, port, quat_delegate=None, debug_delegate=None, data_delegate=None ):
        self.s = serial.Serial(port,921600)
        self.s.setTimeout(0.5)
        self.s.setWriteTimeout(0.2)

        if quat_delegate:
            self.quat_delegate = quat_delegate
        else:
            self.quat_delegate = empty_packet_delegate()

        if debug_delegate:
            self.debug_delegate = debug_delegate
        else:
            self.debug_delegate = empty_packet_delegate()

        if data_delegate:
            self.data_delegate = data_delegate
        else:
            self.data_delegate = empty_packet_delegate()

        self.packets = []
        self.length = 0
        self.previous = None

    def read(self):
        NUM_BYTES = 23
        MAX_PACKET_TYPES = 8
        p = None

        if self.s.inWaiting():
            c = self.s.read(1)
            if ord(c) == ord('$'):
                # Found the start of a valid packet (maybe).
                c = self.s.read(1)
                if ord(c) < MAX_PACKET_TYPES:
                    d = None
                    p = None
                    if ord(c) == 0 or ord(c) == 1 or ord(c) == 7:
                        rs = self.s.read(12)
                        d = data_packet(ord(c),rs)
                    elif ord(c) == 2:
                        rs = self.s.read(16)
                        p = quat_packet(rs)
                        self.quat_delegate.dispatch(p)
                        # Currently, we don't print quaternion data (it's really
                        # meant for the cube display only. If you'd like to
                        # change this behavior, uncomment the following line.
                        #
                        #d = data_packet(ord(c),rs)

                    if d != None:
                        self.data_delegate.dispatch(d)
                else:
                    print "invalid packet type.."

    def write(self,a):
        self.s.write(a)

    def close(self):
        self.s.close()

    def write_log(self,fname):
        f = open(fname,'w')
        for p in self.packets:
            f.write(p.logfile_line())
        f.close()

# ===========  PACKET DELEGATES  ==========
class packet_delegate(object):
    def loop(self,event):
        print "generic packet_delegate loop w/event",event
    def dispatch(self,p):
        print "generic packet_delegate dispatched",p

class empty_packet_delegate(packet_delegate):
    def loop(self,event):
        pass
    def dispatch(self,p):
        pass

class cube_packet_viewer (packet_delegate):
    def __init__(self):
        self.screen = Screen(480,400,scale=1.5)
        self.cube = Cube(30,60,10)
        self.q = Quaternion(1,0,0,0)
        self.previous = None  # previous quaternion
        self.latest = None    # latest packet (get in dispatch, use in loop)

    def loop(self,event):
        packet = self.latest
        if packet:
            q = packet.to_q().normalized()
            self.cube.erase(self.screen)
            self.cube.draw(self.screen,q)
            pygame.display.flip()
            self.latest = None

    def dispatch(self,p):
        if isinstance(p,quat_packet):
            self.latest = p

    def close(self):
        pass


class debug_packet_viewer (packet_delegate):
    def loop(self,event):
        pass

    def dispatch(self,p):
        assert isinstance(p,debug_packet);
        p.display()

class data_packet_viewer (packet_delegate):
    def loop(self,event):
        pass

    def dispatch(self,p):
        assert isinstance(p,data_packet);
        p.display()

# =============== PACKETS ================= 
# For 16-bit signed integers.
def two_bytes(d1,d2):
    d = ord(d1)*256 + ord(d2)
    if d > 32767:
        d -= 65536
    return d

# For 32-bit signed integers.
def four_bytes(d1, d2, d3, d4):
    d = ord(d1)*(1<<24) + ord(d2)*(1<<16) + ord(d3)*(1<<8) + ord(d4)
    if d > 2147483648:
        d-= 4294967296
    return d

class debug_packet (object):
    # body of packet is a debug string
    def __init__(self,l):
        sss = []
        for c in l[3:21]:
            if ord(c) != 0:
                sss.append(c)
        self.s = "".join(sss)

    def display(self):
        sys.stdout.write(self.s)

class data_packet (object):
    def __init__(self, type, l):
        self.data = [0,0,0,0]
        self.type = type
        if self.type == 0:     # accel
            self.data[0] = four_bytes(l[0],l[1],l[2],l[3]) * ACCEL_CONVERSION
            self.data[1] = four_bytes(l[4],l[5],l[6],l[7]) * ACCEL_CONVERSION
            self.data[2] = four_bytes(l[8],l[9],l[10],l[11]) * ACCEL_CONVERSION 
        elif self.type == 1:   # gyro
            self.data[0] = four_bytes(l[0],l[1],l[2],l[3]) * GYRO_CONVERSION
            self.data[1] = four_bytes(l[4],l[5],l[6],l[7]) * GYRO_CONVERSION
            self.data[2] = four_bytes(l[8],l[9],l[10],l[11]) * GYRO_CONVERSION
        elif self.type == 2:   # quaternion
            self.data[0] = four_bytes(l[4],l[5],l[6],l[7]) * QUAT_CONVERSION
            self.data[1] = four_bytes(l[8],l[9],l[10],l[11]) * QUAT_CONVERSION
            self.data[2] = four_bytes(l[12],l[13],l[14],l[15]) * QUAT_CONVERSION
            self.data[3] = four_bytes(l[0],l[1],l[2],l[3]) * QUAT_CONVERSION
        if self.type == 7:     # compass
            self.data[0] = four_bytes(l[0],l[1],l[2],l[3]) * COMPASS_CONVERSION
            self.data[1] = four_bytes(l[4],l[5],l[6],l[7]) * COMPASS_CONVERSION
            self.data[2] = four_bytes(l[8],l[9],l[10],l[11]) * COMPASS_CONVERSION               
        else:   # unsupported
            pass

    def display(self):
        if self.type == 0:
            print 'accel: %7.3f %7.3f %7.3f' % \
                (self.data[0], self.data[1], self.data[2])
        elif self.type == 1:
            print 'gyro: %9.5f %9.5f %9.5f' % \
                (self.data[0], self.data[1], self.data[2])
        elif self.type == 2:
            print 'quat: %7.4f %7.4f %7.4f %7.4f' % \
                (self.data[0], self.data[1], self.data[2], self.data[3])
        elif self.type == 7:
            print 'compass: %9.5f %9.5f %9.5f' % \
                (self.data[0], self.data[1], self.data[2])             
        else:
            print 'what?'
            pass            

class quat_packet (object):
    def __init__(self, l):
        self.l = l
        self.q0 = -(four_bytes(l[0],l[1],l[2],l[3]) * QUAT_CONVERSION)
        self.q1 = four_bytes(l[4],l[5],l[6],l[7]) * QUAT_CONVERSION
        self.q2 = four_bytes(l[8],l[9],l[10],l[11]) * QUAT_CONVERSION
        self.q3 = four_bytes(l[12],l[13],l[14],l[15]) * QUAT_CONVERSION

    def display_raw(self):
        l = self.l
        print "".join(
            [ str(ord(l[0])), " "] + \
            [ str(ord(l[1])), " "] + \
            [ str(ord(a)).ljust(4) for a in 
                                [ l[2], l[3], l[4], l[5], l[6], l[7], l[8], l[9], l[10] ] ] + \
            [ str(ord(a)).ljust(4) for a in 
                                [ l[8], l[9], l[10] , l[11], l[12], l[13]] ]
            )

    def display(self):
        if 1:
            print "qs " + " ".join([str(s).ljust(15) for s in
                [ self.q0, self.q1, self.q2, self.q3 ]])

    def to_q(self):
        return Quaternion(self.q0, self.q1, self.q2, self.q3)

# =============== MAIN ======================
if __name__ == "__main__":
    if len(sys.argv) == 2:
        comport = int(sys.argv[1]) - 1
    else:
        print "usage: " + sys.argv[0] + " port"
        sys.exit(-1)

    pygame.init()
    viewer = cube_packet_viewer()
    debug  = debug_packet_viewer()
    data   = data_packet_viewer()

    reader = motion_driver_packet_reader(comport, 
                quat_delegate = viewer, 
                debug_delegate = debug, 
                data_delegate = data)

    while 1:
        event = pygame.event.poll()
        # TODO: Allow exit via keystroke.
        if event.type == pygame.QUIT:
            viewer.close()
            break
        if event.type == pygame.KEYDOWN:
            reader.write(pygame.key.name(event.key))

        reader.read()
        viewer.loop(event)
        debug.loop(event)
        data.loop(event)

        # TODO: If system load is too high, increase this sleep time.
        pygame.time.delay(0)



