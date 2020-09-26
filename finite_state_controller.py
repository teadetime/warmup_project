#!/usr/bin/env python3
from __future__ import print_function, division
import rospy
import copy
from neato_node.msg import Bump
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from visualization_msgs.msg import Marker
import math
import tty
import select
import sys
import termios
from tf.transformations import euler_from_quaternion
settings = termios.tcgetattr(sys.stdin)
key = None

class tele(object):
    def __init__(self):
        rospy.init_node('obstacle_avoider')
        rospy.Subscriber('/bump', Int8MultiArray, self.process_bump)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        rospy.Subscriber('/odom', Odometry, self.process_odom)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.threshold = .05
        self.front_pt = None
        self.back_pt = None
        self.ranges = None
        self.min_index = None
        self.person_dist = 0
        self.bump = False
        self.state = "start"
        self.dest_x = 3
        self.dest_y = 0
        self.obstacles = None
        self.right_obstacles_dist = None
        self.left_obstacles_dist = None
        self.right_obstacle = False
        self.left_obstacle = False

    def process_scan(self, msg):
        """ Populate the attribute distance_to_wall if we have new data from
            the laser scanner right in front of us """
        # Store ranges from
        if msg.ranges[45] > 0.0:
            self.front_pt  = msg.ranges[45]
        if msg.ranges[135] > 0.0:
            self.back_pt  = msg.ranges[135]
        self.ranges = msg.ranges

    def process_bump(self, msg):
        if any((msg.data[0], msg.data[1], msg.data[2], msg.data[3])):
            self.bump = True

    def convert_pose_to_xy_and_theta(self, pose):
        """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
        orientation_tuple = (pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z,
                             pose.orientation.w)
        angles = euler_from_quaternion(orientation_tuple)
        return (pose.position.x, pose.position.y, angles[2])

    def process_odom(self, msg):
        self.pose = self.convert_pose_to_xy_and_theta(msg.pose.pose)

    def angle_normalize(self, z):
        """ convenience function to map an angle to the range [-pi,pi] """
        return math.atan2(math.sin(z), math.cos(z))

    def angle_diff(self, a, b):
        """ Calculates the difference between angle a and angle b (both should be in radians)
             the difference is always based on the closest rotation from angle a to angle b
            examples:
                angle_diff(.1,.2) -> -.1
                angle_diff(.1, 2*math.pi - .1) -> .2
                angle_diff(.1, .2+2*math.pi) -> -.1
        """
        a = self.angle_normalize(a)
        b = self.angle_normalize(b)
        d1 = a - b
        d2 = 2 * math.pi - math.fabs(d1)
        if d1 > 0:
            d2 *= 1.0
        if math.fabs(d1) < math.fabs(d2):
            return d1
        else:
            return d2

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def get_obstacles(self, threshold, sub_range, full_range):
        #return [i for i, j in enumerate(range) if j < threshold]
        return [full_range.index(i) for i in sub_range if i < threshold]

    def run(self):
        r = rospy.Rate(100)
        self.start_time = rospy.Time.now()
        print(self.start_time, self.start_time + rospy.Duration(1))
        while not rospy.is_shutdown():
            if self.state == "start":
                self.linear_vel = 0
                self.angular_vel = 0.0
                self.linear_vel = 0.2
                if self.ranges is not None:
                    self.state = "avoiding"
            elif self.state == "avoiding":
                if self.bump:
                    print("Yeet")
                    self.state = "stop"
                # Calculate the turn needed to head directly towards dest
                self.target_angle = math.atan2(self.dest_y-self.pose[1], self.dest_x-self.pose[0])
                print(self.target_angle,  self.pose[2])
                self.angle_error = self.angle_diff(self.target_angle, self.pose[2])
                print("Angle to turn...", self.angle_error)
                # Look in the two ranges for the highest values
                self.obstacles = self.get_obstacles(1, self.ranges[45:0:-1] + self.ranges[360:315: -1], self.ranges) #Laser points that are close enough to be detected
                #print(self.obstacles)
                # Check to see if there are obstacles on either side of the robot
                self.left_obstacles = [i for i in self.obstacles if i < 180]
                self.right_obstacles = [i for i in self.obstacles if i > 180]
                if self.right_obstacles:
                    self.right_obstacles_dist = sum(self.ranges[i] for i in self.right_obstacles)/len(self.right_obstacles)
                if self.left_obstacles:
                    self.left_obstacles_dist = sum(self.ranges[i] for i in self.left_obstacles)/len(self.left_obstacles)

                if self.left_obstacles and len(self.left_obstacles) > 2 and self.left_obstacles_dist < .75:
                    # Stuff is in the way go slightly left
                    print("LEFT OBSTACLE STRAIGHT")
                    self.left_obstacle = True
                if self.right_obstacles and len(self.right_obstacles) > 2 and self.right_obstacles_dist < .75:
                    print("RIGHT OBSTACLE STRAIGHT")
                    self.right_obstacle = True

                if self.right_obstacle and self.left_obstacle:
                    print("TW0 Obstacles")
                    # Figure out which object is closer
                    if self.right_obstacles_dist < self.left_obstacles_dist:
                        # move left
                        self.angular_vel = .2 / self.right_obstacles_dist
                    else:
                        # move right
                        self.angular_vel = -.2 / self.left_obstacles_dist
                elif self.right_obstacle:
                    # move left
                    self.angular_vel = .2 / self.right_obstacles_dist
                elif self.left_obstacle:
                    # move right
                    self.angular_vel = -.2 / self.left_obstacles_dist
                else:
                    print("NO OBSTACLES")
                    # No obstacles!!
                    # pointing right towards target
                    self.angular_vel = self.angle_error

                self.right_obstacle = False
                self.left_obstacle = False

                if abs(self.pose[0]-self.dest_x) < .05 and abs(self.pose[1]-self.dest_y) < .05:
                    #Got to destination
                    self.state = "wall_following"
                    self.angular_vel = 0
                    self.linear_vel = .2

            elif self.state == "wall_following":
                if self.bump:
                    print("Yeet")
                    self.state = "stop"
                if self.front_pt > self.back_pt + self.threshold:
                    self.angular_vel = 2 * (self.front_pt - self.back_pt)
                    print("pos angular")
                elif self.back_pt > self.front_pt + self.threshold:
                    self.angular_vel = 2 * (self.front_pt - self.back_pt)
                    print("neg angular")
                else:
                    # Going pretty stright
                    self.angular_vel = 0

            elif self.state == "stop":
                print("Done")
                self.angular_vel = 0
                self.linear_vel = 0
            self.pub.publish(Twist(linear=Vector3(x=self.linear_vel), angular=Vector3(z=self.angular_vel)))
            r.sleep()
        self.pub.publish(Twist(angular=Vector3(z=self.angular_vel)))
        # self.pub.publish(Twist(linear=Vector3(x=0), angular=Vector3(z=0)))


if __name__ == '__main__':
    tele_nathan = tele()
    tele_nathan.run()

