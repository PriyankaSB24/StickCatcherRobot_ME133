
'''sticksim.py

   Simulate a game with randomly falling sticks
   Sticks are arranged in a semicircle in front of Atlas [assuming Atlas is facing +x]
   All sticks must fall before the game resets

   Node:        /sticksim

   Publish:     /stick/pose       geometry_msgs/PoseStamped
                /stick/twist      geometry_msgs/TwistStamped
                /stick/result     std_msgs/String
                /stick/score      std_msgs/Int32
                /stick/markers    visualization_msgs/MarkerArray

   Subscribe:   /hand             geometry_msgs/PoseStamped
                /stick/grab       std_msgs/Bool

   Launch:     Terminal 1:        [stick sim] ros2 run stickcatcher sticksim
               Terminal 2:        [RVIZ] ros2 run rviz2 rviz2 -d ~/robotws/src/stickcatcher/rviz/stickcatcher.rviz
               Terminal 3:        [ATLAS] ros2 run robot_state_publisher robot_state_publisher ~/robotws/src/atlas_description/urdf/atlas_v5.urdf
               Terminal 4:        [Controller] ros2 run stickcatcher catch
               
'''

import rclpy
import numpy as np
import random

from rclpy.node          import Node
from geometry_msgs.msg  import PoseStamped, TwistStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg       import Bool, String, Int32

from stickcatcher.stickpredict import predict

#----------simulation parameters------------------------
DT = 0.01 # simulation time step in seconds
SPAWNZ = 2.20 # height at which sticks are suspended above Atlas

#-------------Stick dimensions & arrangement------------
NSTICKS = 7
LENGTH = 0.25 
RADIUS = 0.018 
ARCRADIUS = 0.55 # radius of the semicircle in front of Atlas

#------------Game timing------------------------------------
WAITMIN = 1.5 # minimum time to wait before releasing the next stick
WAITMAX = 4.0
HOLDTIME = 1.0 # time to hold a caught stick before releasing it
RESETTIME = 5.0 # time to display the final score before starting a new round

# Maximum distance between hand and stick for a catch.
CATCHTOL = 0.08


class StickSim(Node):

    def __init__(self):

        super().__init__('sticksim')

        # arrange sticks in a semicircle from -y to +y
        # passing through +x (in front of Atlas)
        angles = np.radians(np.linspace(-90, 90, NSTICKS))
        self.positions = np.array([[ARCRADIUS * np.cos(a), ARCRADIUS * np.sin(a), SPAWNZ] for a in angles])

        # sticks that have not yet been released
        self.remaining = list(range(NSTICKS))

        # positions of missed sticks on the ground
        self.missed = {}

        # currently active stick
        self.active = None
        self.p = np.zeros(3)
        self.v = np.zeros(3)

        # latest hand position & grabbing command [grab is True when the stick is grabbed... DUH]
        self.phand = None
        self.handtime = None
        self.grab = False
        self.offset = np.zeros(3)

        # game state & timing
        self.state = 'WAIT'
        self.t = 0.0
        self.tstart = 0.0
        self.tdrop = None # time at which the next stick will be released

        # score for the current round
        self.score = 0
        self.attempts = 0
        self.round = 1 # current round number [1 means first round]

        # publishers
        self.pubpose = self.create_publisher(PoseStamped, '/stick/pose', 10)
        self.pubtwist = self.create_publisher(TwistStamped, '/stick/twist', 10)
        self.pubresult = self.create_publisher(String, '/stick/result', 10)
        self.pubscore = self.create_publisher(Int32, '/stick/score', 10)
        self.pubmarkers = self.create_publisher(MarkerArray, '/stick/markers', 10)

        # subscribers
        self.create_subscription(PoseStamped, '/hand', self.recWhand, 10)
        self.create_subscription(Bool, '/stick/grab', self.recgrab, 10)

        # run the simulation at 100 Hz [DT = 0.01 seconds]
        self.start = self.get_clock().now()
        self.timer = self.create_timer(DT, self.update)
        self.get_logger().info("Stick game started.")

    #--------------------ROS callbacks---------------------------------

    # receive the hand position in the world frame
    def recWhand(self, msg):
        p = msg.pose.position # geometry_msgs/Point
        self.phand = np.array([p.x, p.y, p.z])
        self.handtime = self.get_clock().now()

    # receive the robot's grab command
    def recgrab(self, msg):
        self.grab = msg.data

    # schedule the next release after a random waiting time
    def wait(self):
        self.state = 'WAIT'
        self.tdrop = self.t + random.uniform( WAITMIN, WAITMAX)

    #--------------------Game logic/helper functions---------------------

    # randomly release one of the remaining sticks
    def release(self):
        self.active = random.choice(self.remaining)
        self.remaining.remove(self.active)

        self.p = self.positions[self.active].copy()
        self.v = np.zeros(3)
        self.tstart = self.t
        self.state = 'FALL'

        self.grab = False
        self.attempts += 1
        self.get_logger().info("Stick %d/%d released." % (self.attempts, NSTICKS))

        # Send the first measurement immediately after release.
        self.publishstick()

    # check whether the hand is close enough to catch the stick
    def caught(self):
        # ignore if the hand position is unknown or the stick is not being grabbed
        if self.phand is None or not self.grab:
            return False 

        # ignore old hand measurements
        age = (self.get_clock().now() - self.handtime).nanoseconds * 1e-9 # age of the hand measurement in seconds
        if age > 0.10:
            return False

        # find the closest point on the stick's centerline
        closest = self.p.copy()
        closest[2] = np.clip(self.phand[2], # constrain the z-coordinate to the stick's length
                             self.p[2] - LENGTH / 2, # bottom of the stick
                             self.p[2] + LENGTH / 2) # top of the stick
        distance = np.linalg.norm(self.phand - closest)

        return (distance <= CATCHTOL) # [True/False] if the hand is close enough to catch the stick

    # finish/result of the current attempt & update the score
    def finish(self, success):
        self.tstart = self.t # reset the timer for the next state
        self.v = np.zeros(3)

        if success:
            self.state = 'CAUGHT'
            self.score += 1
            # keep the caught stick attached to the hand
            self.offset = self.p - self.phand
            result = 'CAUGHT'
        else:
            self.state = 'MISSED'
            # store the stick's position on the ground
            self.missed[self.active] = self.p.copy()
            result = 'MISSED'

        # publish the result & score & log it
        self.pubresult.publish(String(data=result))
        self.pubscore.publish(Int32(data=self.score))
        self.get_logger().info("%s - Score: %d/%d" % (result, self.score, self.attempts))

    # finish/result of a round after all sticks have fallen
    def finishround(self):
        self.state = 'ROUND_RESET'
        self.tstart = self.t
        self.get_logger().info("Round %d complete! Final score: %d/%d" %(self.round, self.score, NSTICKS))

    # reset/initialize all sticks [as hanging again] & start a new round
    def resetround(self):
        self.remaining = list(range(NSTICKS))
        self.missed = {}
        self.active = None
        self.p = np.zeros(3) # reset the position of the active stick to the origin
        self.v = np.zeros(3) # reset the velocity of the active stick to zero
        self.grab = False
        self.score = 0
        self.attempts = 0
        self.round += 1

        self.pubscore.publish(Int32(data=0))
        self.get_logger().info("Starting round %d." % self.round)
        self.wait()

    #--------------------Publish stick pos & vel---------------------------------

    # publish the active falling stick's position & velocity
    def publishstick(self):
        stamp = self.get_clock().now().to_msg()

        pose = PoseStamped()
        # set the header of the pose
        pose.header.stamp = stamp
        pose.header.frame_id = 'world'
        # set the position & orientation of the stick
        pose.pose.position.x = float(self.p[0])
        pose.pose.position.y = float(self.p[1])
        pose.pose.position.z = float(self.p[2])
        pose.pose.orientation.w = 1.0
        self.pubpose.publish(pose)

        twist = TwistStamped()
        # set the header of the twist [twist = velocity of the stick]
        twist.header = pose.header
        # set the linear velocity of the stick
        twist.twist.linear.x = float(self.v[0])
        twist.twist.linear.y = float(self.v[1])
        twist.twist.linear.z = float(self.v[2])
        self.pubtwist.publish(twist)

    #--------------------Marker visualization in RViz-----------------------------

    # create a cylinder marker representing a stick
    def cylinder(self, index, p, color):
        marker = Marker()

        # set the misc. things needed to add the marker
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'sticks'
        marker.id = index
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # set the position & orientation of the cylinder
        marker.pose.position.x = float(p[0])
        marker.pose.position.y = float(p[1])
        marker.pose.position.z = float(p[2])
        marker.pose.orientation.w = 1.0

        # set the scale & color of the cylinder
        marker.scale.x = 2.0 * RADIUS
        marker.scale.y = 2.0 * RADIUS
        marker.scale.z = LENGTH

        # set the color of the cylinder
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0

        return marker

    # remove a stick marker after it has been caught
    def deletemarker(self, index):
        marker = Marker()

        # set the misc. things needed to delete the marker
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'sticks'
        marker.id = index
        marker.action = Marker.DELETE

        return marker

    # display the current score above the game
    def scoremarker(self):
        marker = Marker()

        # set the misc. things needed to display/add the score marker
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'score'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # set the position & orientation of the score marker
        marker.pose.position.x = 0.55
        marker.pose.position.y = 0.0
        marker.pose.position.z = SPAWNZ + 0.35
        marker.pose.orientation.w = 1.0

        # set the scale & color of the score marker
        marker.scale.z = 0.10
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # set the text of the score marker based on the current state of the game
        if self.state == 'ROUND_RESET':
            # if the round is complete, display the final score
            marker.text = ("Round %d Complete!\nFinal Score: %d/%d" % (self.round, self.score, NSTICKS))
        else:
            # otherwise, display the current score and remaining sticks
            marker.text = ("Round %d\nScore: %d/%d\nRemaining: %d" % (self.round, self.score, NSTICKS, len(self.remaining)))

        return marker

    # display all remaining, falling, and missed sticks
    def publishmarkers(self):
        markers = MarkerArray()

        for i in range(NSTICKS):
            if i == self.active:
                # the currently falling or caught stick
                p = self.p
                if self.state == 'CAUGHT':
                    color = (0.1, 0.9, 0.1)
                elif self.state == 'MISSED':
                    color = (0.5, 0.5, 0.5)
                else: # stick is falling/not yet caught
                    color = (0.9, 0.1, 0.1)
                markers.markers.append(
                    self.cylinder(i, p, color))
                
            elif i in self.remaining:
                # sticks that are still suspended above Atlas
                markers.markers.append(self.cylinder(i, self.positions[i], (0.9, 0.1, 0.1)))

            elif i in self.missed:
                # previously missed sticks remain on the ground
                markers.markers.append(self.cylinder(i, self.missed[i], (0.5, 0.5, 0.5)))

            else:
                # previously caught sticks are removed
                markers.markers.append(self.deletemarker(i))

        markers.markers.append(self.scoremarker())
        self.pubmarkers.publish(markers)

    #--------------------Main simulation loop---------------------------------

    def update(self):
        self.t = (self.get_clock().now() - self.start).nanoseconds * 1e-9

        #---------------------wait for the next random release---------------------
        if self.state == 'WAIT':
            # start the game only after Atlas is online
            if self.phand is not None:
                if self.tdrop is None:
                    self.wait()
                elif self.t >= self.tdrop:
                    self.release()

        #------------------calculate the stick's falling motion-------------------
        elif self.state == 'FALL':
            tau = self.t - self.tstart
            self.p, self.v = predict(self.positions[self.active], np.zeros(3), tau)
            # check whether the stick has hit the ground
            if self.p[2] <= LENGTH / 2:
                self.p[2] = LENGTH / 2
                self.finish(False)
            # check whether Atlas has caught the stick
            elif self.caught():
                self.finish(True)
            else: # stick is still falling, publish its position & velocity
                self.publishstick()

        #----------------stick is caught/missed --> follow the hand-----------------
        elif self.state in ['CAUGHT', 'MISSED']:
            # a caught stick follows the hand temporarily
            if self.state == 'CAUGHT':
                if self.phand is not None:
                    self.p = self.phand + self.offset
            if self.t - self.tstart >= HOLDTIME: # wait for HOLDTIME seconds before releasing the stick
                self.active = None
                self.grab = False
                # continue if there are still suspended sticks
                if len(self.remaining) > 0:
                    self.wait()
                # otherwise --> the round is complete
                else:
                    self.finishround()

        #-------------------reset the round after RESETTIME seconds-------------------
        elif self.state == 'ROUND_RESET':
            if self.t - self.tstart >= RESETTIME:
                self.resetround()

        #-------------------------update the visualization in RViz--------------------
        self.publishmarkers()

# Main code.
def main(args=None):
    rclpy.init(args=args)
    node = StickSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()