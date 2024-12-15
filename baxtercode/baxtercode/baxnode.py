'''GeneratorNode.py

   This creates a trajectory generator node

   from GeneratorNode import GeneratorNode
   generator = GeneratorNode(name, rate, TrajectoryClass)

      Initialize the node, under the specified name and rate.  This
      also requires a trajectory class which must implement:

         trajectory = TrajectoryClass(node)
         jointnames = trajectory.jointnames()
         (q, qdot)  = trajectory.evaluate(t, dt)

      where jointnames, q, qdot are all python lists of the joint's
      name, position, and velocity respectively.  The dt is the time
      since the last evaluation, to be used for integration.

      If trajectory.evaluate() return None, the node shuts down.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

'''

import rclpy
import numpy as np

from asyncio import Future
from rclpy.node import Node
from sensor_msgs.msg import JointState

from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Duration
from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from demos.TransformHelpers import *

import matplotlib.pyplot as plt


#
#   Trajectory Generator Node Class
#
#   This inherits all the standard ROS node stuff, but adds
#     1) an update() method to be called regularly by an internal timer,
#     2) a spin() method, aware when a trajectory ends,
#     3) a shutdown() method to stop the timer.
#
#   Take the node name, the update frequency, and the trajectory class
#   as arguments.
#
class BaxNode(Node):
    # Initialization.
    def __init__(self, name, rate, Trajectory):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Set up a trajectory.
        self.trajectory = Trajectory(self)
        self.jointnames = self.trajectory.jointnames()

        # Add a publisher to send the joint commands.
        self.pub_bax = self.create_publisher(JointState, '/joint_states', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while (not self.count_subscribers('/joint_states')):
            pass

        # Create a future object to signal when the trajectory ends,
        # i.e. no longer returns useful data.

        # Ball node init:

        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             depth=1)
        self.pub = self.create_publisher(
            MarkerArray, '/visualization_marker_array', quality)

        # Initialize the ball position, velocity, set the acceleration.
        self.radius = 0.045

        self.num_balls = 3
        self.positions = []
        # np.array([2.0, 0.0,  5.0       ]
        self.velocities = 3 * [self.num_balls * np.zeros((3, 1))]
        for _ in range(self.num_balls):
            self.positions.append((np.zeros((3, 1))))

        # self.p = np.array([0.0, 0.0, self.radius]).reshape((3,1))
        # self.v = np.array([1.0, 0.1,  5.0       ]).reshape((3,1))
        self.a = np.array([0.0, 0.0, -9.8]).reshape((3, 1))

        # Create the sphere marker.
        diam = 2 * self.radius

        # get_marker()
        # a = 0.8 is slightly transparent!

        # Create the marker array message.
        self.mark = MarkerArray()

        self.count = 0
        self.future = Future()

        # Set up the timing so (t=0) will occur in the first update
        # cycle (dt) from now.
        self.dt = 1.0 / float(rate)
        self.t = -self.dt
        self.start = self.get_clock().now() + rclpy.time.Duration(seconds=self.dt)

        # the time for a throw from the right to the left hand
        self.tRL = 1.2
        # the time for a throw from the left to the right hand
        self.tLR = .4

        # time it takes for a hand to throw a ball
        self.tThrow = .4
        # time it takes for a hand to recover back to catch position
        self.tRecov = .4

        # the delta change in the y and the z direction for the throw to take place
        # for both hands
        self.deltayr = .05
        self.deltazr = .09
        self.deltayl = .09
        self.deltazl = .05

        self.ball_velocity = []
        self.hand_velocity = []
        self.vdrs = []
        self.conds = []
        self.pdrs = []
        self.hand_pos = []
        self.pdls = []

        self.ball_positions = []
        self.left_hand_pos = []

        # Create a timer to keep calculating/sending commands.
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, rate))

    def get_final_ball_loc(self):
        # We want to get the ball to land

        # Right hand ends up with this final velocity / position
        rvf = np.array([0.0, ((2 * self.ystart) - self.deltayr) / self.tRL,
                        (-self.deltazr + (.5 * 9.8 * self.tRL**2)) / self.tRL]).reshape(3, 1)
        rpf = np.array([self.xstart, -self.ystart + self.deltayr,
                       self.zstart + self.deltazr]).reshape(3, 1)

        # Left hand ends up with this final Velocity
        lvf = np.array([0.0, -((2 * self.ystart) - self.deltayl) / self.tLR,
                        (-self.deltazl + (.5 * 9.8 * self.tLR**2)) / self.tLR]).reshape(3, 1)
        lpf = np.array([self.xstart, self.ystart - self.deltayl,
                       self.zstart + self.deltazl]).reshape(3, 1)

        p_ball_right_to_left = rpf + rvf * self.tRL + 1 / 2 * self.a * self.tRL**2

        p_ball_left_to_right = lpf + lvf * self.tLR + 1 / 2 * self.a * self.tLR**2

        return p_ball_right_to_left, p_ball_left_to_right

        # Can we spline from these positions and then Recover back?

        # right/left hand needs to end up here ???
        # rp0 = np.array([self.xstart,-self.ystart,self.zstart]).reshape(3,1)
        # lp0 = np.array([self.xstart,self.ystart,self.zstart]).reshape(3,1)

    def get_marker(self, marker_num, color):
        diam = 2 * self.radius
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.ADD
        marker.ns = "point"
        marker.id = marker_num
        marker.type = Marker.SPHERE
        marker.pose.orientation = Quaternion()
        marker.pose.position = Point_from_p(self.positions[marker_num])
        marker.scale = Vector3(x=diam, y=diam, z=diam)
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.8)
        return marker

    # Shutdown
    def shutdown(self):
        # Destroy the timer, then shut down the node.
        plt.plot(self.ball_velocity)
        plt.title("ball vel")
        plt.show(block=True)
        plt.plot(self.hand_velocity, label = ["x", 'y', 'z'])
        plt.title("right hand velocity")
        plt.show(block=True)
        # plt.plot(self.vdrs)
        # plt.title("desired vel")
        # plt.show(block=True)
        plt.plot(self.conds)
        plt.title("condition num")
        plt.show(block=True)
        plt.plot(self.pdls)
        plt.title("desired pos")
        plt.show()
        plt.plot(self.hand_pos)
        plt.title("hand pos")
        plt.show()

        plt.plot(self.ball_positions)
        plt.title("ball_pos")
        plt.show()

        plt.plot(self.left_hand_pos)
        plt.title("left hand pos")
        plt.show()
        self.timer.destroy()
        self.destroy_node()

    # Spin
    def spin(self):
        # Keep running (taking care of the timer callbacks and message
        # passing), until interrupted or the trajectory is complete
        # (as signaled by the future object).
        rclpy.spin_until_future_complete(self, self.future)

        # Report the reason for shutting down.
        if self.future.done():
            self.get_logger().info("Stopping: " + self.future.result())
        else:
            self.get_logger().info("Stopping: Interrupted")

    def stick_to_hand(self, hand_pos, hand_vel, ball_num):
        closest_hand_idx = np.argmin(
            [
                np.linalg.norm(
                    hand_pos[0] -
                    self.positions[ball_num]),
                np.linalg.norm(
                    hand_pos[1] -
                    self.positions[ball_num])])
        

        self.positions[ball_num] = np.array(
            hand_pos[closest_hand_idx]).copy().reshape(
            (3, 1))
        self.velocities[ball_num] = np.array(hand_vel[closest_hand_idx])

    def stick_to_hand_start(self, hand_pos, hand_vel):
        # Stick to the hand in the beginning

        self.positions[0] = np.array(
            hand_pos[0]).copy().reshape(
            (3, 1))
        self.velocities[0] = np.array(hand_vel[0])
        
        self.positions[1] = np.array(
            hand_pos[1]).copy().reshape(
            (3, 1))
        self.velocities[1] = np.array(hand_vel[1])

        self.positions[1] = np.array(
            hand_pos[0]).copy().reshape(
            (3, 1))
        self.velocities[1] = np.array(hand_vel[0])


    # Update - send a new joint command every time step.
    def update(self):
        # To avoid any time jitter enforce a constant time step and
        # integrate to get the current time.
        self.t += self.dt

        # Determine the corresponding ROS time (seconds since 1970).
        now = self.start + rclpy.time.Duration(seconds=self.t)

        # Compute the desired joint positions and velocities for this time.
        desired, hand_pos, hand_vel, stick, vdr, cond, pdr, pdl, left_stick = self.trajectory.evaluate(
            self.t, self.dt)
        

        if desired is None:
            self.future.set_result("Trajectory has ended")
            return
        self.left_hand_pos.append(hand_pos[1][:, 0])

        self.pdrs.append(pdr.reshape((3)))
        self.pdls.append(pdl.reshape((3)))
        self.hand_pos.append(hand_pos[0][:, 0])
        (q, qdot) = desired

        # Check the results.
        if not (isinstance(q, list) and isinstance(qdot, list)):
            self.get_logger().warn("(q) and (qdot) must be python lists!")
            return
        if not (len(q) == len(self.jointnames)):
            self.get_logger().warn("(q) must be same length as jointnames!")
            return
        if not (len(q) == len(self.jointnames)):
            self.get_logger().warn("(qdot) must be same length as (q)!")
            return
        if not (isinstance(q[0], float) and isinstance(qdot[0], float)):
            self.get_logger().warn("Flatten NumPy arrays before making lists!")
            return

        # Build up a command message and publish.

        colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]

        for i in range(self.num_balls):

            # Integrate the velocity, then the position.
            setup_time = 1
            if self.t > (setup_time + self.tThrow):
                self.velocities[i] = np.copy(
                    self.velocities[i] + self.dt * self.a)
                self.positions[i] = np.copy(
                    self.positions[i] + self.dt * self.velocities[i])

                # if self.positions[i][2, 0] < self.radius:
                #     self.positions[i][2, 0] = np.copy(
                #         self.radius + (self.radius - self.positions[i][2, 0]))
                #     self.velocities[i] = np.zeros((3, 1))
                        # np.copy(self.velocities[i][2, 0])

            have_close = (
                np.all(
                    np.isclose(
                        self.positions[i],
                        hand_pos[0], atol=.2)) or np.all(
                    np.isclose(
                        self.positions[i],
                        hand_pos[1], atol=.2)))
            
            self.ball_velocity.append(self.velocities[0][0, 0])
            
            self.hand_velocity.append(hand_vel[0][:,0])

            closest_hand_idx = np.argmin(
            [
                np.linalg.norm(
                    hand_pos[0] -
                    self.positions[i]),
                np.linalg.norm(
                    hand_pos[1] -
                    self.positions[i])])

            if left_stick and closest_hand_idx == 1:
                self.stick_to_hand(hand_pos, hand_vel, i)

            if self.t <= (setup_time + self.tThrow):
                self.stick_to_hand_start(hand_pos, hand_vel)
            elif self.t <= (setup_time + self.tThrow + self.tRecov):
                # we want one of the balls to stick initially
                self.stick_to_hand(hand_pos, hand_vel, 0)
            if (stick and have_close):
                self.stick_to_hand(hand_pos, hand_vel, i)


            self.vdrs.append(vdr[0, 0])
            # self.stick_to_hand(hand_pos, hand_vel, i)
            # # Check for a bounce - not the change in x velocity is non-physical.

            marker = self.get_marker(i, colors[i])

            # We add the new marker to the MarkerArray, removing the oldest
            # marker from it when necessary
            if (self.count + 1 > self.num_balls):
                self.mark.markers.pop(0)
            else:
                self.count += 1
            self.mark.markers.append(marker)
            self.conds.append(cond)

            self.ball_positions.append(self.positions[i][:, 0])


            # now = self.start + Duration(seconds=self.t)
            # reset marker ids
            mark_id = 0
            for mark in self.mark.markers:
                mark.id = mark_id
                mark_id += 1
                mark.header.stamp  = now.to_msg()

        cmdmsg = JointState()
        cmdmsg.header.stamp = now.to_msg()      # Current time for ROS
        cmdmsg.name = self.jointnames   # List of joint names
        cmdmsg.position = q                 # List of joint positions
        cmdmsg.velocity = qdot              # List of joint velocities
        self.pub_bax.publish(cmdmsg)
        self.pub.publish(self.mark)
