'''catch.py
   Trajectory generation node 
   Catch stick that is dropped within reach of its right arm, pelvis to right arm 
   Node:        /trajectory
   Publish:     /joint_states           sensor_msgs/JointState
                /hand                   geometry_msgs/PoseStamped
                (world -> pelvis TF)    tf2_ros broadcast
   Subscribe:   /stick/pose             geometry_msgs/PoseStamped
'''

import rclpy
import numpy as np
import tf2_ros
from math               import pi, sin, cos, acos, atan2, sqrt, exp, fmod
from asyncio            import Future
from rclpy.node         import Node
from geometry_msgs.msg  import PoseStamped, TwistStamped
from geometry_msgs.msg  import TransformStamped
from sensor_msgs.msg    import JointState
from std_msgs.msg       import Header, Bool, String

from utils.TransformHelpers        import *
from utils.TrajectoryUtils         import goto, goto5, spline
from stickcatcher.KinematicChain     import KinematicChain


PELVISZ = 0.95  #pelvis z so feet touch ground
ZCATCH = 1.05   # catch height
TREACH = 0.45   # time to reach catch point (s)
REACH = 0.75    #  max arm reach (m)
SPAWNZ = 2.20   # spawn z (matches stick.py)


# Trajectory Generator Node  +   update()
class TrajectoryNode(Node): 
    def __init__(self, name, future): 
        super().__init__(name)
        self.future = future

        # Check with atlas urdf  
        self.armjoints = ['r_arm_shz', 'r_arm_shx', 'r_arm_ely',
                          'r_arm_elx', 'r_arm_wry', 'r_arm_wrx',
                          'r_arm_wry2']

        #  Check with atlas urdf  
        self.fixedjoints = {
            'back_bkz':  0.0, 'back_bky':  0.0, 'back_bkx':  0.0,
            'neck_ry':   0.0,
            'l_arm_shz': 0.0, 'l_arm_shx': -1.2, 'l_arm_ely': 1.6,
            'l_arm_elx': 1.2, 'l_arm_wry':  0.0, 'l_arm_wrx': 0.0,
            'l_arm_wry2': 0.0,
            'l_leg_hpz': 0.0, 'l_leg_hpx': 0.0, 'l_leg_hpy': -0.2,
            'l_leg_kny': 0.4, 'l_leg_aky': -0.2, 'l_leg_akx': 0.0,
            'r_leg_hpz': 0.0, 'r_leg_hpx': 0.0, 'r_leg_hpy': -0.2,
            'r_leg_kny': 0.4, 'r_leg_aky': -0.2, 'r_leg_akx': 0.0,
        }

        # Arm kin chain
        self.chain = KinematicChain(self, 'utorso', 'r_hand', self.armjoints)

        # The fixed world -> pelvis transform 
        self.Rwp = Reye()
        self.pwp = pxyz(0.0, 0.0, PELVISZ)

        # pelvis to upper torso transformation
        # all three back joint angles are zero
        self.ppt = pxyz(-0.0125, 0.0, 0.212)

        # world to upper torso transformation
        self.Rwt = self.Rwp
        self.pwt = self.pwp + self.Rwp @ self.ppt

        self.q0 = np.radians(np.array([0.0, 60.0, 90.0, -90.0,
                                       0.0, 0.0, 0.0]))
        self.qc = self.q0
        (self.phome, _, _, _) = self.chain.fkin(self.q0)

        # Move the hand from pA to pB 
        self.pA     = self.phome
        self.pB     = self.phome
        self.tstart = 0.0

        # Latest stick position (world frame), and the previous height
        self.pstick = None
        self.zlast  = 0.0

        self.lam  = 20.0
        self.lams = 10.0

        # Node logistics
        self.pubjoint = self.create_publisher(JointState, '/joint_states', 10)
        self.pubhand  = self.create_publisher(PoseStamped, '/hand', 10)
        self.pubgrab = self.create_publisher(Bool, '/stick/grab', 10)

        # Broadcast the world -> pelvis transform.
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(PoseStamped, '/stick/pose', self.recvstick, 10)
        self.create_subscription(String, '/stick/result', self.recvresult, 10)
        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass

        # 100 hz 
        self.dt    = 0.01                       # 100Hz.
        self.t     = -self.dt                   # Seconds since start
        self.now   = self.get_clock().now()      
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, 1/self.dt))

    # Shutdown
    def shutdown(self): 
        self.timer.destroy()
        self.destroy_node()


    # get stick
    def recvstick(self, msg):
        self.pstick = p_from_T(T_from_Pose(msg.pose))

        # restart again for new stick drop
        if (self.pstick[2] > SPAWNZ - 0.05) and (self.zlast < SPAWNZ - 0.05):
            (p, _, _, _) = self.chain.fkin(self.qc)
            self.pA     = p
            self.pB     = self.catchpoint(self.pstick)
            self.tstart = self.t
            self.get_logger().info(
                "New drop - reaching to pelvis-frame (%.2f, %.2f, %.2f)" %
                tuple(self.pB))

        self.zlast = self.pstick[2]

    # return the hand home after a successful or missed catch
    def recvresult(self, msg):
        if msg.data in ['CAUGHT', 'MISSED']:
            (p, _, _, _) = self.chain.fkin(self.qc)
            self.pA = p
            self.pB = self.phome.copy()
            self.tstart = self.t
            self.pstick = None
            self.zlast = 0.0
            self.get_logger().info("%s - Returning home." % msg.data)

    # world stick pos -> pelvis for reaching 
    def catchpoint(self, pstick):
        # Desired catching point in the world frame
        pw = pxyz(pstick[0], pstick[1], ZCATCH)

        # convert to the upper torso frame
        pd = self.Rwt.T @ (pw - self.pwt)

        # approximate reach limit.
        d = pd - self.phome
        if np.linalg.norm(d) > REACH:
            pd = self.phome + d * REACH / np.linalg.norm(d)
        return pd


    # Update 
    def update(self): 
        self.t   = self.t   + self.dt
        self.now = self.now + rclpy.time.Duration(seconds=self.dt)

        # Get trajectory
        # (1) The desired hand position/velocity (pelvis frame). 
        # Spline from pA to pB, then be ocnst at pB
        tseg = self.t - self.tstart
        if tseg < TREACH:
            (pd, vd) = goto5(tseg, TREACH, self.pA, self.pB)
        else:
            (pd, vd) = (self.pB, vzero())

        # forward kin
        (p, R, Jv, Jw) = self.chain.fkin(self.qc)

        # inverse kin, pull hand back 
        e     = ep(pd, p)
        xdot  = vd + self.lam * e

        qdots = self.lams * (self.q0 - self.qc)
        qdot  = qdots + np.linalg.pinv(Jv) @ (xdot - Jv @ qdots)

        # integrate
        self.qc = self.qc + self.dt * qdot

        # Report if we are not converging  
        if np.linalg.norm(e) > 0.5:
            self.get_logger().warn("Tip error %.3f m" % np.linalg.norm(e))

        # publish data
        header = Header(stamp=self.now.to_msg(), frame_id='world')
        names = self.armjoints      + list(self.fixedjoints.keys())
        pos   = self.qc.tolist()    + list(self.fixedjoints.values())
        vel   = qdot.tolist()       + [0.0] * len(self.fixedjoints)

        self.pubjoint.publish(JointState(
            header=header, name=names, position=pos, velocity=vel))

        # world -> pelvis transform 
        trans = Transform_from_Rp(self.Rwp, self.pwp)
        self.broadcaster.sendTransform(TransformStamped(
            header=header, child_frame_id='pelvis', transform=trans))

        # The hand pose, in the WORLD frame
        (p, R, _, _) = self.chain.fkin(self.qc)
        pwh = self.pwt + self.Rwt @ p
        Rwh = self.Rwt @ R
        self.pubhand.publish(PoseStamped(
            header=header, pose=Pose_from_Rp(Rwh, pwh)))
        
        # attempt to grab near the end of the reaching trajectory
        grab = self.pstick is not None and tseg >= TREACH - 0.10
        self.pubgrab.publish(Bool(data=bool(grab)))

#  Main Code
def main(args=None):
    rclpy.init(args=args)
    future = Future()
    trajectory = TrajectoryNode('trajectory', future)

    # Keeps running
    rclpy.spin_until_future_complete(trajectory, future)

    # error
    if future.done():
        trajectory.get_logger().info("Stopping: " + future.result())
    else:
        trajectory.get_logger().info("Stopping: Interrupted")
    trajectory.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
