'''stick.py

   This is the "world" node for the ME/CS/EE 133a final project.  It
   owns the falling stick: it spawns the stick at a random location
   above the robot, drops it under gravity, and (once the hand gets
   close enough) kinematically attaches it to the hand.

   The stick is NOT part of the robot URDF.  It is drawn as a single
   CYLINDER marker, so we are free to place it anywhere we like.

   Node:        /stick
   Publish:     /stick/pose             geometry_msgs/PoseStamped
                /stick/twist            geometry_msgs/TwistStamped
                /visualization_marker_array
                                        visualization_msgs/MarkerArray
   Subscribe:   /hand                   geometry_msgs/PoseStamped

'''

import rclpy
import numpy as np

from math                   import pi, sin, cos, acos, atan2, sqrt, exp, fmod

from rclpy.node             import Node
from rclpy.qos              import QoSProfile, DurabilityPolicy
from geometry_msgs.msg      import PoseStamped, TwistStamped
from std_msgs.msg           import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from utils.TransformHelpers        import *


#
#   Stick Parameters
#
GRAVITY   = 9.81            # m/s^2
LENGTH    = 0.60            # Stick length (m)
RADIUS    = 0.02            # Stick radius (m)

SPAWNZ    = 2.20            # Height at which the stick appears (m)
CATCHR    = 0.12            # Hand must be this close to grab (m)

TWAIT     = 1.0             # Pause before the first/next drop (s)
THOLD     = 2.0             # How long to hold the stick after a catch (s)
ZFLOOR    = 0.0             # Below this, the stick has been missed (m)


#
#   Stick Node Class
#
#   Same structure as the homework trajectory nodes: an __init__() to
#   set things up, an update() called by a timer at 100Hz, and a
#   shutdown().  The only difference is that we integrate a falling
#   object instead of computing joint angles.
#
class StickNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node.
        super().__init__(name)

        ##############################################################
        # INITIALIZE THE STICK DATA!

        # The stick state: center position, velocity, orientation.
        self.p    = pzero()
        self.v    = vzero()
        self.R    = Reye()

        # The current mode: 'WAIT', 'FALL', or 'HELD'.
        self.mode  = 'WAIT'
        self.tmode = 0.0            # Time at which we entered the mode

        # The latest hand pose (filled in by the subscriber).
        self.phand = None
        self.Rhand = Reye()

        # Random number generator.  FIXME: fix the seed while you are
        # debugging, so every run drops the stick the same way!
        self.rng = np.random.default_rng(seed=133)

        ##############################################################
        # Setup the logistics of the node:
        # Add the publishers.  The markers are latched (transient
        # local) so RVIZ sees them even if it connects late.
        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             depth=1)
        self.pubmark  = self.create_publisher(
            MarkerArray, '/visualization_marker_array', quality)
        self.pubpose  = self.create_publisher(PoseStamped,  '/stick/pose',  10)
        self.pubtwist = self.create_publisher(TwistStamped, '/stick/twist', 10)

        # Subscribe to the hand pose, published by the catch node.
        self.create_subscription(PoseStamped, '/hand', self.recvhand, 10)

        # Create the marker we will keep re-publishing.
        self.marker                 = Marker()
        self.marker.header.frame_id = 'world'
        self.marker.ns              = 'stick'
        self.marker.id              = 1
        self.marker.type            = Marker.CYLINDER
        self.marker.action          = Marker.ADD
        self.marker.scale.x         = 2 * RADIUS
        self.marker.scale.y         = 2 * RADIUS
        self.marker.scale.z         = LENGTH
        self.marker.color           = ColorRGBA(r=1.0, g=0.65, b=0.0, a=1.0)
        self.markerarray            = MarkerArray(markers=[self.marker])

        # Set up the timer to update at 100Hz, with (t=0) occuring in
        # the first update cycle (dt) from now.
        self.dt    = 0.01                       # 100Hz.
        self.t     = -self.dt                   # Seconds since start
        self.now   = self.get_clock().now()     # ROS time since 1970
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("Stick running with dt of %f seconds (%fHz)" %
                               (self.dt, 1/self.dt))

    # Shutdown
    def shutdown(self):
        # Destroy the timer, then shut down the node.
        self.timer.destroy()
        self.destroy_node()


    # Receive the hand pose from the catch node.
    def recvhand(self, msg):
        self.phand = p_from_T(T_from_Pose(msg.pose))
        self.Rhand = R_from_T(T_from_Pose(msg.pose))


    # Pick a new random drop.  Keep it inside the right arm's reach so
    # that (for now) the robot can catch it WITHOUT stepping.
    # FIXME: once the stepping works, widen these ranges!
    def spawn(self):
        x = self.rng.uniform( 0.25, 0.55)       # In front of the robot
        y = self.rng.uniform(-0.55,-0.15)       # Toward the right arm
        tilt = self.rng.uniform(-0.3, 0.3)      # Small random lean
        yaw  = self.rng.uniform(-pi,  pi)

        self.p = pxyz(x, y, SPAWNZ)
        self.v = vzero()
        self.R = Rotz(yaw) @ Roty(tilt)
        self.get_logger().info("Dropping stick at (%.2f, %.2f, %.2f)" %
                               (x, y, SPAWNZ))


    # Update - advance the stick and publish, every time step.
    def update(self):
        # Increment time.  We do so explicitly to avoid system jitter.
        self.t   = self.t   + self.dt
        self.now = self.now + rclpy.time.Duration(seconds=self.dt)

        ##############################################################
        # ADVANCE THE STICK AT THIS TIME INSTANCE.

        if self.mode == 'WAIT':
            # Sit still, then drop.
            self.v = vzero()
            if self.t - self.tmode > TWAIT:
                self.spawn()
                self.mode  = 'FALL'
                self.tmode = self.t

        elif self.mode == 'FALL':
            # Simple ballistic integration (Euler is plenty at 100Hz).
            self.v = self.v + self.dt * vxyz(0.0, 0.0, -GRAVITY)
            self.p = self.p + self.dt * self.v

            # Did the hand reach it?  This is the "kinematic grab":
            # we just check distance, no contact dynamics.
            caught = ((self.phand is not None) and
                      (np.linalg.norm(self.phand - self.p) < CATCHR))

            if caught:
                self.get_logger().info("CAUGHT at t = %.2f" % self.t)
                self.mode  = 'HELD'
                self.tmode = self.t
            elif self.p[2] < ZFLOOR:
                # Or did we miss it entirely?
                self.get_logger().info("MISSED at t = %.2f" % self.t)
                self.mode  = 'WAIT'
                self.tmode = self.t

        elif self.mode == 'HELD':
            # The stick is rigidly attached to the hand - its pose is
            # simply the hand's pose.  Nothing to integrate.
            if self.phand is not None:
                self.p = self.phand
                self.R = self.Rhand
            self.v = vzero()
            if self.t - self.tmode > THOLD:
                self.mode  = 'WAIT'
                self.tmode = self.t

        ##############################################################
        # Finish by publishing the data (pose, twist, and marker).
        header = Header(stamp=self.now.to_msg(), frame_id='world')

        self.pubpose.publish(PoseStamped(
            header=header, pose=Pose_from_Rp(self.R, self.p)))

        self.pubtwist.publish(TwistStamped(
            header=header, twist=Twist_from_vw(self.v, vzero())))

        self.marker.header.stamp = self.now.to_msg()
        self.marker.pose         = Pose_from_Rp(self.R, self.p)
        self.pubmark.publish(self.markerarray)


#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the stick node.
    stick = StickNode('stick')

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted.
    rclpy.spin(stick)

    # Shutdown the node and ROS.
    stick.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
