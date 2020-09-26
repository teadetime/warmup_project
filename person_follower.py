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
settings = termios.tcgetattr(sys.stdin)
key = None

class tele(object):
    def __init__(self):
        rospy.init_node('person_follower')
        rospy.Subscriber('/bump', Int8MultiArray, self.process_bump)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
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

    def process_scan(self, msg):
        """ Populate the attribute distance_to_wall if we have new data from
            the laser scanner right in front of us """
        # Store ranges from
        self.ranges = msg.ranges

    def process_bump(self, msg):
        if any((msg.data[0], msg.data[1], msg.data[2], msg.data[3])):
            self.bump = True

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        r = rospy.Rate(10)
        self.start_time = rospy.Time.now()
        print(self.start_time, self.start_time + rospy.Duration(1))
        while not rospy.is_shutdown():
            if self.state == "start":
                self.linear_vel = 0
                self.angular_vel = 0
                self.linear_vel = 0
                if self.ranges is not None:
                    self.state = "following"
            elif self.state == "following":
                if self.bump:
                    print("Yeet")
                    self.state = 'stop'
                # Look in the two ranges for the highest values
                self.min_index = self.ranges.index(min(self.ranges[90:0:-1] + self.ranges[360:270: -1]))
                self.person_dist = self.ranges[self.min_index]
                print(self.min_index, self.ranges[self.min_index])
                if(self.person_dist != "inf"):
                    self.linear_vel = self.person_dist/2
                    if self.min_index > 180:
                        self.min_index = self.min_index-360
                    self.angular_vel = self.min_index/45
                else:
                    self.angular_vel = .1 # Turn to find something!!
                    self.linear_vel = 0
            elif self.state == "stop":
                self.angular_vel = 0
                self.linear_vel = 0
            self.pub.publish(Twist(linear=Vector3(x=self.linear_vel),angular=Vector3(z=self.angular_vel)))
            r.sleep()
        self.pub.publish(Twist(angular=Vector3(z=self.angular_vel)))
        # self.pub.publish(Twist(linear=Vector3(x=0), angular=Vector3(z=0)))


if __name__ == '__main__':
    tele_nathan = tele()
    tele_nathan.run()

