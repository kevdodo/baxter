# '''baxhand_demo.py

#    Node:        /generator
#    Publish:     /joint_states           sensor_msgs/JointState

# '''
import threading
import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Import the format for the condition number message
from std_msgs.msg import Float64

# Grab the utilities
from hw5code.GeneratorNode      import GeneratorNode
from hw5code.TransformHelpers   import *
from hw5code.TrajectoryUtils    import *

# Grab the general fkin from HW5 P5.
from hw5code.KinematicChain     import KinematicChain
from baxtercode.baxnode import BaxNode

# #
# #   Trajectory Class
# #
class Trajectory():
  # Initialization.
  def __init__(self, node):
    # a chain is made for the right hand
    self.chain1 = KinematicChain(node, 'world', 'rh_palm', self.jointnames1())
    # followed by the left hand
    self.chain2 = KinematicChain(node, 'world', 'lh_palm', self.jointnames2())
    self.pub = node.create_publisher(Float64, '/condition', 10)
    # init the joints to all zero as the start position
    # the right / left arm / hand will each work with a subsection of this q
    self.q0  = np.zeros(63)
    self.q = self.q0

    # determine the start position in task space for the right arm
    (self.p0r, self.R0r, _, _) = self.chain1.fkin(self.q0[1:10])
    self.pr = self.p0r
    self.Rr = self.R0r

    # determine the start position in task space for the left arm
    (self.p0l, self.R0l, _, _) = self.chain2.fkin(self.q0[32:41])
    self.pl = self.p0l
    self.Rl = self.R0l

  # Declare the joint names.
  # all of the join names
  def jointnames(self):
    # Return a list of joint names
    return ['head_pan',\
            'right_s0','right_s1','right_e0','right_e1',\
            'right_w0','right_w1','right_w2',\
            'rh_WRJ2','rh_WRJ1',\
            'rh_FFJ1','rh_FFJ2','rh_FFJ3','rh_FFJ4',\
            'rh_MFJ1','rh_MFJ2','rh_MFJ3','rh_MFJ4',\
            'rh_RFJ1','rh_RFJ2','rh_RFJ3','rh_RFJ4',\
            'rh_LFJ1','rh_LFJ2','rh_LFJ3','rh_LFJ4','rh_LFJ5',\
            'rh_THJ1','rh_THJ2','rh_THJ3','rh_THJ4','rh_THJ5',\
            'left_s0','left_s1','left_e0','left_e1',\
            'left_w0','left_w1','left_w2',\
            'lh_WRJ2','lh_WRJ1',\
            'lh_FFJ1','lh_FFJ2','lh_FFJ3','lh_FFJ4',\
            'lh_MFJ1','lh_MFJ2','lh_MFJ3','lh_MFJ4',\
            'lh_RFJ1','lh_RFJ2','lh_RFJ3','lh_RFJ4',\
            'lh_LFJ1','lh_LFJ2','lh_LFJ3','lh_LFJ4','lh_LFJ5',\
            'lh_THJ1','lh_THJ2','lh_THJ3','lh_THJ4','lh_THJ5']
  # joint names for the right and left arm. The ordering of the list matters, 
  # must match the URDF
  def jointnames1(self):
    # Return a list of joint names
    return ['right_s0','right_s1','right_e0','right_e1',
            'right_w0','right_w1','right_w2', 'rh_WRJ2', 'rh_WRJ1']
  def jointnames2(self):
    # Return a list of joint names
    return [
            'left_s0','left_s1','left_e0','left_e1',\
            'left_w0','left_w1','left_w2','lh_WRJ2','lh_WRJ1']
  # Evaluate at the given time.  This was last called (dt) ago.
  def evaluate(self, t, dt):
    # spend 3 seconds moving arms from there initial position to the desired 
    # demo position
    if t < 3:
      pdr, vdr = goto(t, 3, self.p0r, np.array([0.7,-0.7,0.0]).reshape(3,1))
      Rdr = self.R0r
      wdr = np.array([0,0,0]).reshape(3,1)
      pdl, vdl = goto(t, 3, self.p0l, np.array([0.7,0.7,0.0]).reshape(3,1))
      Rdl = self.R0l
      wdl = np.array([0,0,0]).reshape(3,1)

    # move and rotate the arms cyclically 
    elif t < np.inf:
      s = sin(t-3)
      sdot = cos(t-3)
      # just movements up and down
      pdr = np.array([0.7,-0.7,.5*s]).reshape(3,1) 
      vdr = np.array([0.0,0.0,.5*sdot]).reshape(3,1)
      # rotating in the same direction as Identity but not all the way 
      Rdr = Rote(np.array([.707,.707,0]),(pi/2)-((pi/2)-(pi*s/4)))@Rotz(((pi/2)+(pi/4)-s*(pi/2)))
      wdr = exyz(1,1,0)*(pi*sdot/4)+Rote(np.array([.707,.707,0]),(pi/2)-((pi/2)-(pi*s/4)))@(ez()*-sdot*(pi/2))

      pdl = np.array([0.7,0.7,.5*s]).reshape(3,1) 
      vdl = np.array([0.0,0.0,.5*sdot]).reshape(3,1)
      Rdl = self.R0l
      wdl = np.array([0,0,0]).reshape(3,1)
    else:
      return None

    print(f'vdr: {vdr}')
    print(f'wdr: {wdr}')    
    (pr, Rr, Jvr, Jwr) = self.chain1.fkin(self.q[1:10])

    # print("right hand before", pr)
    Jr = np.vstack((Jvr, Jwr)) 
    e = np.vstack((ep(self.pr, pr), eR(self.Rr, Rr)))
    gamma = .1
    Jpr = np.transpose(Jr)@np.linalg.inv(Jr@np.transpose(Jr)+gamma**2*np.eye(6))
    qdotr =  Jpr @ (np.vstack((vdr, wdr)) + (20 * e))
    qr = self.q[1:10].reshape(9,1)  + dt * qdotr.reshape(9,1) 

    (pl, Rl, Jvl, Jwl) = self.chain2.fkin(self.q[32:41])
    Jl = np.vstack((Jvl, Jwl)) 
    e = np.vstack((ep(self.pl, pl), eR(self.Rl, Rl)))
    gamma = .1
    Jpl = np.transpose(Jl)@np.linalg.inv(Jl@np.transpose(Jl)+gamma**2*np.eye(6))
    qdotl =  Jpl @ (np.vstack((vdl, wdl)) + (20 * e))
    ql = self.q[32:41].reshape(9,1)  + dt * qdotl.reshape(9,1) 

    q = np.zeros(63)
    q[0] = sin(t)
    qdot = np.zeros(63)
    qdot[0] = cos(t)

    q[1:10] = qr.flatten()
    qdot[1:10] = qdotr.flatten()

    q[32:41] = ql.flatten()
    qdot[32:41] = qdotl.flatten()

    self.q = q
    self.Rr = Rdr
    self.pr = pdr

    self.Rl = Rdl
    self.pl = pdl

    (pr_new, Rr_new, Jvr, Jwr) = self.chain1.fkin(self.q[1:10])

    (pl_new, Rl_new, Jvl, Jwl) = self.chain2.fkin(self.q[32:41])

    joint_out = q.flatten().tolist(), qdot.flatten().tolist()

    positions = pr_new.flatten().tolist(), pl_new.flatten().tolist()
    # print("right hand after", pr_new)

    velocities = Jvr @ qdotr , Jvl @ qdotl

    return joint_out, positions, velocities

#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the demo node (100Hz).
    rclpy.init(args=args)

    # generator = GeneratorNode('generator', 100, Trajectory)
    # Run until interrupted.
    # generator.spin()

    node = BaxNode('baxter', 100, Trajectory)
    node.spin()
    # Shutdown the node and ROS.

    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
