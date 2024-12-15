# '''baxhand_demo.py
# '''
import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Import the format for the condition number message
from std_msgs.msg import Float64
# Grab the utilities
from hw5code.TransformHelpers import *
from hw5code.TrajectoryUtils import *

# Grab the general fkin from HW5 P5.
from hw5code.KinematicChain import KinematicChain
from baxtercode.baxnode import BaxNode

# #
# #   Trajectory Class
# #


class Trajectory():
    # Initialization.
    def __init__(self, node):
        # a chain is made for the right hand
        self.chain1 = KinematicChain(
            node, 'world', 'rh_palm', self.jointnames1())
        # followed by the left hand
        self.chain2 = KinematicChain(
            node, 'world', 'lh_palm', self.jointnames2())
        # init the joints to all zero as the start position
        # the right / left arm / hand will each work with a subsection of this
        # q
        self.q0 = np.zeros(63)
        self.q = self.q0
        self.right_srt = 1
        self.right_end = 10
        self.left_srt = 32
        self.left_end = 41
        # determine the start position in task space for the right arm
        (self.p0r, self.R0r, _, _) = \
            self.chain1.fkin(self.q0[self.right_srt:self.right_end])
        self.pr = self.p0r
        self.Rr = self.R0r
        # determine the start position in task space for the left arm
        (self.p0l, self.R0l, _, _) = \
            self.chain2.fkin(self.q0[self.left_srt:self.left_end])
        self.pl = self.p0l
        self.Rl = self.R0l

        # constants to define motion
        self.xstart = .8
        self.ystart = .8
        self.zstart = .3
        self.setup_time = 1
        

        # These are the constants for juggling, it is set up so that the time of
        # flight for the top ball is 3x the time of flight for the bottom ball and
        # the time it takes for the hands to throw the ball is equal to the time
        # of the bottom balls flight, and this is also equal to the recover time
        # this makes for a seamless catch throw recover repeat cycle

        # the time for a throw from the right to the left hand
        self.tRL = 1.2
        # the time for a throw from the left to the right hand
        self.tLR = (self.tRL / 3) - .05

        # time it takes for a hand to throw a ball
        self.tThrow = self.tRL / 3
        # time it takes for a hand to recover back to catch position
        self.tRecov = self.tRL / 3

        # the delta change in the y and the z direction for the throw to take place
        # for both hands
        self.deltayr = .05
        self.deltazr = .09
        self.deltayl = .09
        self.deltazl = .05

        self.ball_final = np.zeros((3, 1)), np.zeros((3, 1))

        self.prev_stick = False
        self.last_hand_pos = None

        self.ball_pos_next = None


    # Declare the joint names.
    # all of the join names
    def jointnames(self):
        # Return a list of joint names
        return ['head_pan',
                'right_s0', 'right_s1', 'right_e0', 'right_e1',
                'right_w0', 'right_w1', 'right_w2',
                'rh_WRJ2', 'rh_WRJ1',
                'rh_FFJ1', 'rh_FFJ2', 'rh_FFJ3', 'rh_FFJ4',
                'rh_MFJ1', 'rh_MFJ2', 'rh_MFJ3', 'rh_MFJ4',
                'rh_RFJ1', 'rh_RFJ2', 'rh_RFJ3', 'rh_RFJ4',
                'rh_LFJ1', 'rh_LFJ2', 'rh_LFJ3', 'rh_LFJ4', 'rh_LFJ5',
                'rh_THJ1', 'rh_THJ2', 'rh_THJ3', 'rh_THJ4', 'rh_THJ5',
                'left_s0', 'left_s1', 'left_e0', 'left_e1',
                'left_w0', 'left_w1', 'left_w2',
                'lh_WRJ2', 'lh_WRJ1',
                'lh_FFJ1', 'lh_FFJ2', 'lh_FFJ3', 'lh_FFJ4',
                'lh_MFJ1', 'lh_MFJ2', 'lh_MFJ3', 'lh_MFJ4',
                'lh_RFJ1', 'lh_RFJ2', 'lh_RFJ3', 'lh_RFJ4',
                'lh_LFJ1', 'lh_LFJ2', 'lh_LFJ3', 'lh_LFJ4', 'lh_LFJ5',
                'lh_THJ1', 'lh_THJ2', 'lh_THJ3', 'lh_THJ4', 'lh_THJ5']
    # joint names for the right and left arm. The ordering of the list matters,
    # must match the URDF

    def jointnames1(self):
        # Return a list of joint names
        return [
            'right_s0', 'right_s1', 'right_e0', 'right_e1',
            'right_w0', 'right_w1', 'right_w2', 'rh_WRJ2', 'rh_WRJ1']

    def jointnames2(self):
        # Return a list of joint names
        return [
            'left_s0', 'left_s1', 'left_e0', 'left_e1',
            'left_w0', 'left_w1', 'left_w2', 'lh_WRJ2', 'lh_WRJ1']

    # this will set the hands from their initial position to their start of juggle
    # position
    def gotostart(self, t, T):
        # the path variables used for right hand
        s, sdot = goto5(t, T, 0, (pi / 4))
        g, gdot = goto5(t, T, 0, (pi / 2))
        # right hand position, orientation, and (angular) velocities
        pdr, vdr = goto5(t, T, self.p0r, np.array(
            [self.xstart, -self.ystart, self.zstart]).reshape(3, 1))
        Rdr = Rote(exyz(1, 1, 0), pi / 2) @ Rotz((pi / 2) +
                                                 (pi / 4)) @ Rotx(s) @ Rotz(g)
        wdr = ex() * sdot + Rotx(s) @ ez() * gdot

        # the path variables used for left hand
        s, sdot = goto5(t, T, 0, (pi / 4))
        g, gdot = goto5(t, T, 0, (-pi / 2))
        # left hand position, orientation, and (angular) velocities
        pdl, vdl = goto(t, T, self.p0l, np.array(
            [self.xstart, self.ystart, self.zstart]).reshape(3, 1))
        Rdl = Rote(exyz(-1, 1, 0), (pi / 2)
                   ) @ Rotz((pi / 4)) @ Rotx(s) @ Rotz(g)
        wdl = ex() * sdot + Rotx(s) @ ez() * gdot

        return pdr, vdr, Rdr, wdr, pdl, vdl, Rdl, wdl

    # the robot needs to perform 1 throw with just the right hand to begin the
    # juggle. This will move the hand by delta y and delta z and will have a
    # velocity calculated to move the ball to the next hand in self.tRL time
    def rightThrow(self, t, T):
        p0 = np.array([self.xstart, -self.ystart, self.zstart]).reshape(3, 1)
        pf = np.array([self.xstart, -self.ystart + self.deltayr,
                      self.zstart + self.deltazr]).reshape(3, 1)
        v0 = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
        vf = np.array([0, ((2 * self.ystart) - self.deltayr) / self.tRL,
                       (-self.deltazr + (.5 * 9.8 * self.tRL**2)) / self.tRL]).reshape(3, 1)
        a = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
        s, sdot = goto5(
            t, T, 0, -((pi / 2) - atan2(self.deltazr, self.deltayr)))

        pdr, vdr = spline5(t, T, p0, pf, v0, vf, a, a)
        Rdr = Rote(exyz(1, 1, 0), pi / 2) @ Rotz((pi / 2) + (pi / 4)
                                                 ) @ Rotx(pi / 4) @ Rotz(pi / 2) @ Rotz(s)
        wdr = ez() * sdot

        pdl = np.array([self.xstart, self.ystart, self.zstart]).reshape(3, 1)
        vdl = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
        Rdl = Rote(exyz(-1, 1, 0), (pi / 2)) @ Rotz((pi / 4)
                                                    ) @ Rotx(pi / 4) @ Rotz(-pi / 2)
        wdl = np.array([0.0, 0.0, 0.0]).reshape(3, 1)

        return pdr, vdr, Rdr, wdr, pdl, vdl, Rdl, wdl

    # this function will bring the ball from the throw position back to the start
    # position ready to catch and throw another ball
    def rightRecover(self, t, T):
        pf = np.array([self.xstart, -self.ystart, self.zstart]).reshape(3, 1)
        p0 = np.array([self.xstart, -self.ystart + self.deltayr,
                      self.zstart + self.deltazr]).reshape(3, 1)
        vf = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
        v0 = np.array([0.0, ((2 * self.ystart) - self.deltayr) / self.tRL,
                      (-self.deltazr + (.5 * 9.8 * self.tRL**2)) / self.tRL]).reshape(3, 1)
        a = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
        s, sdot = goto5(
            t, T, -((pi / 2) - atan2(self.deltazr, self.deltayr)), 0)

        pdr, vdr = spline5(t, T, p0, pf, v0, vf, a, a)
        Rdr = Rote(exyz(1, 1, 0), pi / 2) @ Rotz((pi / 2) + (pi / 4)
                                                 ) @ Rotx(pi / 4) @ Rotz(pi / 2) @ Rotz(s)
        wdr = ez() * sdot

        pdl = np.array([self.xstart, self.ystart, self.zstart]).reshape(3, 1)
        vdl = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
        Rdl = Rote(exyz(-1, 1, 0), (pi / 2)) @ Rotz((pi / 4)
                                                    ) @ Rotx(pi / 4) @ Rotz(-pi / 2)
        wdl = np.array([0.0, 0.0, 0.0]).reshape(3, 1)

        return pdr, vdr, Rdr, wdr, pdl, vdl, Rdl, wdl

    # these next two functions are similar to the above 2 but they throw and
    # recover both hands
    def throw(self, t, T):
        p0 = np.array([self.xstart, -self.ystart, self.zstart]).reshape(3, 1)
        pf = np.array([self.xstart, -self.ystart + self.deltayr,
                      self.zstart + self.deltazr]).reshape(3, 1)
        v0 = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
        vf = np.array([0.0, ((2 * self.ystart) - self.deltayr) / self.tRL,
                       (-self.deltazr + (.5 * 9.8 * self.tRL**2)) / self.tRL]).reshape(3, 1)
        a = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
        s, sdot = goto5(
            t, T, 0, -((pi / 2) - atan2(self.deltazr, self.deltayr)))

        pdr, vdr = spline5(t, T, p0, pf, v0, vf, a, a)
        Rdr = Rote(exyz(1, 1, 0), pi / 2) @ Rotz((pi / 2) + (pi / 4)
                                                 ) @ Rotx(pi / 4) @ Rotz(pi / 2) @ Rotz(s)
        wdr = ez() * sdot

        p0 = np.array([self.xstart, self.ystart, self.zstart]).reshape(3, 1)
        pf = np.array([self.xstart, self.ystart - self.deltayl,
                      self.zstart + self.deltazl]).reshape(3, 1)
        v0 = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
        vf = np.array([0.0, -((2 * self.ystart) - self.deltayl) / self.tLR,
                       (-self.deltazl + (.5 * 9.8 * self.tLR**2)) / self.tLR]).reshape(3, 1)
        a = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
        s, sdot = goto5(t, T, 0, (pi / 2) - atan2(self.deltazl, self.deltayl))

        pdl, vdl = spline5(t, T, p0, pf, v0, vf, a, a)
        Rdl = Rote(exyz(-1, 1, 0), (pi / 2)) @ Rotz((pi / 4)
                                                    ) @ Rotx(pi / 4) @ Rotz(-pi / 2) @ Rotz(s)
        wdl = ez() * sdot

        return pdr, vdr, Rdr, wdr, pdl, vdl, Rdl, wdl
    
    def recover_ball(self, t, T):
        left_hand_pos_end, right_hand_pos_end = self.ball_pos_next

        # spline to these positions based off of last positions
        right_hand_start, left_hand_start = self.last_hand_pos

        pdr, vdr = goto5(t, T, right_hand_start, right_hand_pos_end)

        s, sdot = goto5(
            t, T, -((pi / 2) - atan2(self.deltazr, self.deltayr)), 0)

        Rdr = Rote(exyz(1, 1, 0), pi / 2) @ Rotz((pi / 2) + (pi / 4)
                                                 ) @ Rotx(pi / 4) @ Rotz(pi / 2) @ Rotz(s)
        wdr = ez() * sdot

        # left hand splines
        pdl, vdl = goto5(t, T, left_hand_start, left_hand_pos_end)

        s, sdot = goto5(t, T, (pi / 2) - atan2(self.deltazl, self.deltayl), 0)
        
        Rdl = Rote(exyz(-1, 1, 0), (pi / 2)) @ Rotz((pi / 4)
                                                    ) @ Rotx(pi / 4) @ Rotz(-pi / 2) @ Rotz(s)
        wdl = ez() * sdot

        return pdr, vdr, Rdr, wdr, pdl, vdl, Rdl, wdl
    
    def go_to_start_recover(self, t, T):
        left_hand_pos_end, right_hand_pos_end = self.ball_pos_next

        wdr = np.zeros((3, 1))

        Rdr = Rote(exyz(1, 1, 0), pi / 2) @ Rotz((pi / 2) + (pi / 4)
                                                 ) @ Rotx(pi / 4) @ Rotz(pi / 2)
        
        p0r = right_hand_pos_end

        pfr = np.array([self.xstart, -self.ystart, self.zstart]).reshape(3, 1)
        pdr , vdr = goto5(t, T, p0r, pfr)

        # left hand
        p0l = left_hand_pos_end

        pfl = np.array([self.xstart, self.ystart, self.zstart]).reshape(3, 1)

        wdl = np.zeros((3, 1))
        Rdl = Rote(exyz(-1, 1, 0), (pi / 2)) @ Rotz((pi / 4)
                                                    ) @ Rotx(pi / 4) @ Rotz(-pi / 2)
        
        pdl , vdl = goto5(t, T, p0l, pfl)
        return pdr, vdr, Rdr, wdr, pdl, vdl, Rdl, wdl


    # TODO: Rename to go to start
    def recover(self, t, T):
        pf = np.array([self.xstart, -self.ystart, self.zstart]).reshape(3, 1)
        p0 = np.array([self.xstart, -self.ystart + self.deltayr,
                      self.zstart + self.deltazr]).reshape(3, 1)
        vf = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
        v0 = np.array([0.0, ((2 * self.ystart) - self.deltayr) / self.tRL,
                      (-self.deltazr + (.5 * 9.8 * self.tRL**2)) / self.tRL]).reshape(3, 1)
        a = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
        s, sdot = goto5(
            t, T, -((pi / 2) - atan2(self.deltazr, self.deltayr)), 0)

        pdr, vdr = spline5(t, T, p0, pf, v0, vf, a, a)
        Rdr = Rote(exyz(1, 1, 0), pi / 2) @ Rotz((pi / 2) + (pi / 4)
                                                 ) @ Rotx(pi / 4) @ Rotz(pi / 2) @ Rotz(s)
        wdr = ez() * sdot

        pf = np.array([self.xstart, self.ystart, self.zstart]).reshape(3, 1)
        p0 = np.array([self.xstart, self.ystart - self.deltayl,
                      self.zstart + self.deltazl]).reshape(3, 1)
        vf = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
        v0 = np.array([0.0, -((2 * self.ystart) - self.deltayl) / self.tLR,
                       (-self.deltazl + (.5 * 9.8 * self.tLR**2)) / self.tLR]).reshape(3, 1)
        a = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
        s, sdot = goto5(t, T, (pi / 2) - atan2(self.deltazl, self.deltayl), 0)

        pdl, vdl = spline5(t, T, p0, pf, v0, vf, a, a)
        Rdl = Rote(exyz(-1, 1, 0), (pi / 2)) @ Rotz((pi / 4)
                                                    ) @ Rotx(pi / 4) @ Rotz(-pi / 2) @ Rotz(s)
        wdl = ez() * sdot

        return pdr, vdr, Rdr, wdr, pdl, vdl, Rdl, wdl

    def holdStart(self):
        pdr = np.array([self.xstart, -self.ystart, self.zstart]).reshape(3, 1)
        vdr = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
        Rdr = Rote(exyz(1, 1, 0), pi / 2) @ Rotz((pi / 2) + \
                   (pi / 4)) @ Rotx(pi / 4) @ Rotz(pi / 2)
        wdr = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
        pdl = np.array([self.xstart, self.ystart, self.zstart]).reshape(3, 1)
        vdl = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
        Rdl = Rote(exyz(-1, 1, 0), (pi / 2)) @ Rotz((pi / 4)
                                                    ) @ Rotx(pi / 4) @ Rotz(-pi / 2)
        wdl = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
        return pdr, vdr, Rdr, wdr, pdl, vdl, Rdl, wdl
    
    def holdThrow(self):
        pdr = np.array([self.xstart, -self.ystart + self.deltayr,
                      self.zstart + self.deltazr]).reshape(3, 1)
        vdr = np.array([0.0, 0.0, 0.0]).reshape(3, 1)

        # pdr, vdr = spline5(t, T, p0, pf, v0, vf, a, a)
        Rdr = Rote(exyz(1, 1, 0), pi / 2) @ Rotz((pi / 2) + (pi / 4)
                                                 ) @ Rotx(pi / 4) @ Rotz(pi / 2) @ Rotz((pi / 2) - atan2(self.deltazr, self.deltayr))
        wdr = ez() * 0

        pdl = np.array([self.xstart, self.ystart - self.deltayl,
                      self.zstart + self.deltazl]).reshape(3, 1)
        vdl = np.array([0.0, 0.0,0.0]).reshape(3, 1)
    
        Rdl = Rote(exyz(-1, 1, 0), (pi / 2)) @ Rotz((pi / 4)
                                                    ) @ Rotx(pi / 4) @ Rotz(-pi / 2) @ Rotz((pi / 2) - atan2(self.deltazl, self.deltayl))
        wdl = ez() * 0
        return pdr, vdr, Rdr, wdr, pdl, vdl, Rdl, wdl
    
    def get_ball_pos_end(init_vel, t, a, x0):
        x = x0 + init_vel * t
        x[2] += 1 / 2 * t * a


    def get_final_ball_loc(self, rvf, rpf, lvf, lpf):
        # We want to get the ball to land

        # Right hand ends up with this final velocity / position
        # rvf = np.array([0.0, ((2 * self.ystart) - self.deltayr) / self.tRL,
        #                 (-self.deltazr + (.5 * 9.8 * self.tRL**2)) / self.tRL]).reshape(3, 1)
        # rpf = np.array([self.xstart, -self.ystart + self.deltayr,
        #                self.zstart + self.deltazr]).reshape(3, 1)

        # # Left hand ends up with this final Velocity
        # lvf = np.array([0.0, -((2 * self.ystart) - self.deltayl) / self.tLR,
        #                 (-self.deltazl + (.5 * 9.8 * self.tLR**2)) / self.tLR]).reshape(3, 1)
        # lpf = np.array([self.xstart, self.ystart - self.deltayl,
        #                self.zstart + self.deltazl]).reshape(3, 1)
        accel = np.zeros((3, 1))
        accel[2, 0] = -9.8

        t = self.tThrow + self.tRecov + self.tRecov - .075

        p_ball_right_to_left = rpf + rvf * t + 1 / 2 * accel * t**2
        p_ball_left_to_right = lpf + lvf * t + 1 / 2 * accel * t**2

        return p_ball_right_to_left, p_ball_left_to_right        

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        # first the hands are sent to the start
        stick = False

        base_case_stupid = False
        left_stick = False
        # if t >= self.setup_time:
        #     return None, None, None, None, None, None, None, None, None


        if t <= self.setup_time:
            pdr, vdr, Rdr, wdr, pdl, vdl, Rdl, wdl = self.gotostart(
                t, self.setup_time)
            stick = True
        elif t <= self.setup_time + self.tThrow:
            # return None, None, None, None, None, None, None
            pdr, vdr, Rdr, wdr, pdl, vdl, Rdl, wdl = self.rightThrow(
                t - self.setup_time, self.tThrow)
            stick = True
            base_case_stupid = True
            # return None, None, None, None, None, None, None
        # elif t <= self.setup_time + self.tThrow:
        #     pdr, vdr, Rdr, wdr, pdl, vdl, Rdl, wdl = self.holdThrow()
        # elif t <= self.setup_time + self.tThrow + self.tRecov:
        #     pdr, vdr, Rdr, wdr, pdl, vdl, Rdl, wdl = self.go_to_start_recover(t, .05)
        elif t <= self.setup_time + self.tThrow + self.tRecov:
            # return None, None, None, None, None, None, None
            left_stick = True
            pdr, vdr, Rdr, wdr, pdl, vdl, Rdl, wdl = self.rightRecover(
                t - self.setup_time - self.tThrow, self.tRecov)
        else:
            # return None, None, None, None, None, None, None
            t = t - self.setup_time - self.tThrow - self.tRecov
            t = t % (self.tThrow + self.tRecov)
            if t <= self.tThrow:
                pdr, vdr, Rdr, wdr, pdl, vdl, Rdl, wdl = self.throw(
                    t, (self.tThrow))
                stick = True
            # elif t <= self.setup_time + self.tThrow:
            #     pdr, vdr, Rdr, wdr, pdl, vdl, Rdl, wdl = self.holdThrow()
            elif t <= self.tThrow + self.tRecov - .1:
                # self.ball_queue.pop(0)
                pdr, vdr, Rdr, wdr, pdl, vdl, Rdl, wdl = self.recover_ball(t - self.tThrow,
                                                                        self.tRecov - .1)
            elif t <= self.tThrow + self.tRecov:
                # return None, None, None, None, None, None, None, None

                stick = True
                pdr, vdr, Rdr, wdr, pdl, vdl, Rdl, wdl = self.go_to_start_recover(t - (self.tThrow + self.tRecov - .1), .1)

                # pdr, vdr, Rdr, wdr, pdl, vdl, Rdl, wdl = self.recover(
                #     t - self.tThrow, self.tRecov)
                
            # else:
            #     return None, None, None, None, None, None, None, None, None

        # inverse kin for the right hand
        (pr, Rr, Jvr, Jwr) = self.chain1.fkin(
            self.q[self.right_srt:self.right_end])
        Jr = np.vstack((Jvr, Jwr))
        # e = np.vstack((ep(self.pr, pr), eR(self.Rr, Rr)))
        e = np.vstack((ep(self.pr, pr), eR(self.Rr, Rr)))
        gamma = .1
        lam = 80

        # gamma = 0.1

        # qsdot = np.zeros((7,1))
        # qsdot[3,0] = lams * (-np.pi/2 - qlast[3,0])

        qsec = .5 * np.array([(.61 - self.q[1]), (.95 - self.q[2]), (.43 - self.q[3]), (-1.1 - self.q[4]),
                        (.77 - self.q[5]), (-.64 - self.q[6]), (.68 - self.q[7]), (-1.0 - self.q[8]), 
                        (-.17 - self.q[9])]).reshape(9, 1)
        Jrinv = Jr.T @ np.linalg.pinv(Jr @ Jr.T + gamma**2 * np.eye(6))

        # xrdot = np.vstack((vdr, wdr))


        # qdotr = Jrinv @ xrdot + (np.eye(9) - Jrinv @ Jr) @ qsec
        # qr = self.q[self.right_srt:self.right_end].reshape(9, 1) + dt * qdotr.reshape(9, 1)

        Jpr = np.transpose(Jr) @ np.linalg.inv(Jr @ np.transpose(Jr) + gamma**2 * np.eye(6))\
            + ((np.eye(9) - np.linalg.pinv(Jr) @ Jr) @
               qsec)
        qdotr = Jpr @ (np.vstack((vdr, wdr)) + (lam * e))
        qr = self.q[self.right_srt:self.right_end].reshape(
            9, 1) + dt * qdotr.reshape(9, 1)

        # inverse kin for left hand
        (pl, Rl, Jvl, Jwl) = self.chain2.fkin(
            self.q[self.left_srt:self.left_end])
        Jl = np.vstack((Jvl, Jwl))
        e = np.vstack((ep(self.pl, pl), eR(self.Rl, Rl)))

        Jpl = np.transpose(Jl) @ np.linalg.inv(Jl @ np.transpose(Jl) + gamma**2 * np.eye(6))\
            + ((np.eye(9) - np.linalg.pinv(Jl) @ Jl) @
               (.5 * np.array([(-.61 - self.q[32]), (.95 - self.q[33]), (-.43 - self.q[34]), (-1.1 - self.q[35]), (-.77 - self.q[36]),
                               (-.64 - self.q[37]), (-.68 - self.q[38]), (-1.0 - self.q[39]), (-.70 - self.q[40])])).reshape(9, 1))
        qdotl = Jpl @ (np.vstack((vdl, wdl)) + (lam * e))

        cond_num = np.linalg.cond(Jpl)

        ql = self.q[self.left_srt:self.left_end].reshape(
            9, 1) + dt * qdotl.reshape(9, 1)

        # set the new joint positions and velocities
        q = np.zeros(63)
        qdot = np.zeros(63)
        q[self.right_srt:self.right_end] = qr.flatten()
        qdot[self.right_srt:self.right_end] = qdotr.flatten()
        q[self.left_srt:self.left_end] = ql.flatten()
        qdot[self.left_srt:self.left_end] = qdotl.flatten()

        # save the positions for the next iteration
        self.q = q
        self.Rr = Rdr
        self.pr = pdr
        self.Rl = Rdl
        self.pl = pdl

        # get the new position to send to the ball node
        (pr_new, _, Jvr, Jwr) = self.chain1.fkin(
            self.q[self.right_srt:self.right_end])
        (pl_new, _, Jvl, Jwl) = self.chain2.fkin(
            self.q[self.left_srt:self.left_end])
        # flatten the joints and the position and velocities so that the bax node
        # can update and update the balls
        joint_out = q.flatten().tolist(), qdot.flatten().tolist()
        positions = pr_new.reshape(3, 1), pl_new.reshape(3, 1) # pr.reshape(3, 1), pl.reshape(3, 1)
        velocities = Jvr @ qdotr, Jvl @ qdotl

        if base_case_stupid:
            p_ball_right_to_left, p_ball_left_to_right = self.get_final_ball_loc(velocities[0], positions[0], velocities[1], positions[1])
            self.ball_pos_next = p_ball_right_to_left, np.array([self.xstart, -self.ystart, self.zstart]).reshape(3, 1)

        self.last_hand_pos = positions
        # eif self.prev_stick and not stick:
        #     self.ball_pos_next = self.get_final_ball_loc(velocities[0], positions[0], velocities[1], positions[1])

        self.prev_stick = stick

        return joint_out, positions, velocities, stick, vdr, cond_num, pdr, pdl, left_stick

#
#  Main Code
#


def main(args=None):
    
    # Initialize ROS and the demo node (100Hz).
    rclpy.init(args=args)

    # generator = GeneratorNode('generator', 100, Trajectory)
    # Run until interrupted.
    # generator.spin()

    node = BaxNode('baxter', 500, Trajectory)
    node.spin()
    # Shutdown the node and ROS.

    node.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
