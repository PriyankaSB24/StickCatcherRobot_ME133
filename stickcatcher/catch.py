'''catch.py

   Final project, milestone 1: Atlas stands still and catches a stick
   that gets dropped somewhere random in front of its right arm.

   The idea: the stick is dropped from rest, so its x and y never
   change on the way down - only z does.  So the arm doesn't have to
   chase anything.  It just figures out where the stick is going to
   land, splines the hand over to (x, y, catch height), and waits
   there for the stick to show up.  Way easier than tracking it.

   We only run IK on the right arm (pelvis -> r_hand, 7 joints).
   Everything else in the robot is just frozen at a fixed pose, and
   the pelvis is bolted to the world.  No walking yet - that's
   milestone 2.

   Node:        /trajectory
   Publish:     /joint_states           sensor_msgs/JointState
                /hand                   geometry_msgs/PoseStamped
                (world -> pelvis TF)    tf2_ros broadcast
   Subscribe:   /stick/pose             geometry_msgs/PoseStamped
                /robot_description      std_msgs/String

'''

import rclpy
import numpy as np
import tf2_ros
import xml.etree.ElementTree as ET

from math               import pi, sin, cos, acos, atan2, sqrt, exp, fmod

from asyncio            import Future
from rclpy.node         import Node
from rclpy.qos          import QoSProfile, DurabilityPolicy
from geometry_msgs.msg  import PoseStamped, TwistStamped
from geometry_msgs.msg  import TransformStamped
from sensor_msgs.msg    import JointState
from std_msgs.msg       import Header, String

from utils.TransformHelpers        import *
from utils.TrajectoryUtils         import goto, goto5, spline
from stickcatcher.KinematicChain   import KinematicChain


#
#   Numbers to play with
#
#   These are the only things worth tuning.  PELVISZ is the one that
#   actually has to be right or the robot floats / sinks into the floor.
#
PELVISZ = 0.95          # how high the pelvis sits so the feet touch z=0
ZCATCH  = 1.05          # height we try to catch at (comfy arm height)
TREACH  = 0.80          # seconds to swing the hand over to the catch spot
REACH   = 0.75          # how far the hand can get from home before we give up
SPAWNZ  = 2.20          # has to match stick.py! this is how we notice a new drop


#
#   The posture we freeze the rest of the body in.
#
#   Any movable joint in the URDF that isn't a right-arm joint gets
#   held at whatever value is listed here.  Anything not listed just
#   sits at 0.  So you do NOT have to list all 20-something Atlas
#   joints - only the ones where 0 looks silly (the knees, mostly,
#   otherwise he stands bolt upright like a plank).
#
POSTURE = {
    'l_arm_shx': -1.2, 'l_arm_ely':  1.6, 'l_arm_elx':  1.2,
    'l_leg_hpy': -0.2, 'l_leg_kny':  0.4, 'l_leg_aky': -0.2,
    'r_leg_hpy': -0.2, 'r_leg_kny':  0.4, 'r_leg_aky': -0.2,
}


#
#   Trajectory Generator Node Class
#
#   Same shape as every homework node: __init__() sets stuff up,
#   update() runs at 100Hz and does the actual work, shutdown() kills
#   the timer.
#
class TrajectoryNode(Node):
    # Initialization.
    def __init__(self, name, future):
        # Initialize the node and store the future object (to end).
        super().__init__(name)
        self.future = future

        ##############################################################
        # INITIALIZE YOUR TRAJECTORY DATA!

        # The 7 right arm joints - these are the actual DOFs we solve
        # for.  If the node dies on startup complaining it can't find
        # these, the names are wrong for our URDF and the error
        # message prints the real ones, so just copy them in.
        self.armjoints = ['r_arm_shz', 'r_arm_shx', 'r_arm_ely',
                          'r_arm_elx', 'r_arm_wry', 'r_arm_wrx',
                          'r_arm_wry2']

        # Grab every movable joint name straight out of the URDF, so
        # we don't have to type them all out by hand.  Everything
        # that isn't an arm joint gets frozen at its POSTURE value.
        alljoints = self.readjointnames()
        self.fixedjoints = {n: POSTURE.get(n, 0.0)
                            for n in alljoints if n not in self.armjoints}

        # Sanity check - way nicer than a confusing crash later.
        missing = [n for n in self.armjoints if n not in alljoints]
        if missing:
            self.get_logger().error("Can't find these joints: %s" % missing)
            self.get_logger().error("The URDF actually has: %s" % alljoints)
            raise RuntimeError("Fix self.armjoints in catch.py")

        # Set up the kinematic chain for the right arm.
        self.chain = KinematicChain(
            self, 'pelvis', 'r_hand', self.armjoints)

        # Where the pelvis sits.  Atlas isn't allowed to move yet, so
        # this never changes - we just broadcast it every cycle so
        # RVIZ knows where to draw him.
        self.Rwp = Reye()
        self.pwp = pxyz(0.0, 0.0, PELVISZ)

        # Starting arm pose, and the joint command we carry between
        # cycles (this is the q_c from the inverse kinematics notes).
        self.q0 = np.radians(np.array([0.0, 60.0, 90.0, -90.0,
                                       0.0, 0.0, 0.0]))
        self.qc = self.q0

        # Where the hand starts out, in the pelvis frame.
        (self.phome, _, _, _) = self.chain.fkin(self.q0)

        # The current spline segment: hand goes pA -> pB over TREACH.
        # Both start at home so it just sits still until a stick shows up.
        self.pA     = self.phome
        self.pB     = self.phome
        self.tstart = 0.0

        # Latest stick position, plus its last height so we can tell
        # when a brand new stick has been spawned.
        self.pstick = None
        self.zlast  = 0.0

        # Convergence gain, and the gain on the secondary task.
        self.lam  = 20.0
        self.lams = 10.0

        ##############################################################
        # Setup the logistics of the node:
        # Add a publisher to send the joint commands and the hand pose.
        self.pubjoint = self.create_publisher(JointState, '/joint_states', 10)
        self.pubhand  = self.create_publisher(PoseStamped, '/hand', 10)

        # Broadcast the world -> pelvis transform.
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # Listen for the stick.
        self.create_subscription(PoseStamped, '/stick/pose',
                                 self.recvstick, 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass

        # Set up the timer to update at 100Hz, with (t=0) occuring in
        # the first update cycle (dt) from now.
        self.dt    = 0.01                       # 100Hz.
        self.t     = -self.dt                   # Seconds since start
        self.now   = self.get_clock().now()     # ROS time since 1970
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, 1/self.dt))

    # Shutdown
    def shutdown(self):
        # Destroy the timer, then shut down the node.
        self.timer.destroy()
        self.destroy_node()


    # Pull the movable joint names out of the URDF.
    #
    # robot_state_publisher dumps the whole URDF onto
    # /robot_description as one big string, and it's latched, so we
    # can just wait for it and parse the XML.  Saves typing out every
    # Atlas joint by hand and getting one of them subtly wrong.
    def readjointnames(self):
        self.urdf = None
        def cb(msg):
            self.urdf = msg.data
        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             depth=1)
        sub = self.create_subscription(
            String, '/robot_description', cb, quality)

        self.get_logger().info("Waiting for the URDF...")
        while self.urdf is None:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)

        # A joint is a real DOF if it isn't "fixed".
        return [j.get('name') for j in ET.fromstring(self.urdf).findall('joint')
                if j.get('type') != 'fixed']


    # Stick showed up - store it, and notice if it's a brand new drop.
    def recvstick(self, msg):
        self.pstick = p_from_T(T_from_Pose(msg.pose))

        # If the stick just teleported back up to the spawn height,
        # that's a new throw.  Start a fresh spline from wherever the
        # hand happens to be right now.
        if (self.pstick[2] > SPAWNZ - 0.05) and (self.zlast < SPAWNZ - 0.05):
            (p, _, _, _) = self.chain.fkin(self.qc)
            self.pA     = p
            self.pB     = self.catchpoint(self.pstick)
            self.tstart = self.t
            self.get_logger().info(
                "New drop - heading to (%.2f, %.2f, %.2f) in pelvis frame" %
                tuple(self.pB))

        self.zlast = self.pstick[2]


    # Work out where to put the hand, given where the stick is.
    #
    # Since the stick is dropped from rest, x and y are already final
    # the instant it spawns - we only have to pick the height.  So the
    # "prediction" is basically free right now.
    #
    # FIXME (Jenny): when the stick gets thrown sideways instead of
    # dropped, this is the function that needs the real ballistic
    # solve - find t* where z(t*) = ZCATCH, then evaluate x(t*), y(t*).
    def catchpoint(self, pstick):
        # Where we want the hand, in world coordinates.
        pw = pxyz(pstick[0], pstick[1], ZCATCH)

        # Convert into the pelvis frame, since that's where the chain lives.
        pd = self.Rwp.T @ (pw - self.pwp)

        # If it's out past what the arm can reach, just go as far as we
        # can in that direction instead of asking for the impossible
        # (which makes the IK thrash around near the singularity).
        d = pd - self.phome
        if np.linalg.norm(d) > REACH:
            pd = self.phome + d * REACH / np.linalg.norm(d)
        return pd


    # Update - send a new joint command every time step.
    def update(self):
        # Increment time.  We do so explicitly to avoid system jitter.
        self.t   = self.t   + self.dt
        self.now = self.now + rclpy.time.Duration(seconds=self.dt)

        ##############################################################
        # COMPUTE THE TRAJECTORY AT THIS TIME INSTANCE.

        # Step 1: where should the hand be right now?  Quintic spline
        # over to the catch point, then just park there.
        tseg = self.t - self.tstart
        if tseg < TREACH:
            (pd, vd) = goto5(tseg, TREACH, self.pA, self.pB)
        else:
            (pd, vd) = (self.pB, vzero())

        # Step 2: forward kinematics at the last joint command.
        (p, R, Jv, Jw) = self.chain.fkin(self.qc)

        # Step 3: the actual inverse kinematics.  Only 3 rows of task
        # (just position, we don't care how the wrist is twisted yet)
        # but 7 joints, so the arm is redundant and the pseudo-inverse
        # has a nullspace.  We use it for a secondary task that gently
        # pulls the arm back toward its starting pose, otherwise the
        # elbow drifts off into weird configurations over time.
        e     = ep(pd, p)
        xdot  = vd + self.lam * e

        qdots = self.lams * (self.q0 - self.qc)
        qdot  = qdots + np.linalg.pinv(Jv) @ (xdot - Jv @ qdots)

        # Step 4: integrate to get the new joint command.
        self.qc = self.qc + self.dt * qdot

        # If the error is big something is wrong - probably asking for
        # a point outside the workspace.  Yell about it.
        if np.linalg.norm(e) > 0.5:
            self.get_logger().warn("Tip error %.3f m" % np.linalg.norm(e))

        ##############################################################
        # Finish by publishing the data.
        header = Header(stamp=self.now.to_msg(), frame_id='world')

        # The arm joints we solved for, plus every other joint frozen.
        names = self.armjoints      + list(self.fixedjoints.keys())
        pos   = self.qc.tolist()    + list(self.fixedjoints.values())
        vel   = qdot.tolist()       + [0.0] * len(self.fixedjoints)

        self.pubjoint.publish(JointState(
            header=header, name=names, position=pos, velocity=vel))

        # Tell RVIZ where the pelvis is.  (Delete this in milestone 2!
        # Once the URDF has the 6 base joints, robot_state_publisher
        # does this itself and the two will fight each other.)
        trans = Transform_from_Rp(self.Rwp, self.pwp)
        self.broadcaster.sendTransform(TransformStamped(
            header=header, child_frame_id='pelvis', transform=trans))

        # Send the hand pose in WORLD coords, because that's the frame
        # the stick node thinks in.
        (p, R, _, _) = self.chain.fkin(self.qc)
        pwh = self.pwp + self.Rwp @ p
        Rwh = self.Rwp @ R
        self.pubhand.publish(PoseStamped(
            header=header, pose=Pose_from_Rp(Rwh, pwh)))


#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Create a future object to signal when the trajectory ends.
    future = Future()

    # Initialize the trajectory generator node.
    trajectory = TrajectoryNode('trajectory', future)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory is
    # complete (as signaled by the future object).
    rclpy.spin_until_future_complete(trajectory, future)

    # Report the reason for shutting down.
    if future.done():
        trajectory.get_logger().info("Stopping: " + future.result())
    else:
        trajectory.get_logger().info("Stopping: Interrupted")

    # Shutdown the node and ROS.
    trajectory.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
