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
        rospy.init_node('wall_follower')
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.threshold = .05
        self.front_pt = None
        self.back_pt = None

        self.state = "start"
        self.moving_towards_wall = None
        self.distance_target = None

    def process_scan(self, msg):
        """ Populate the attribute distance_to_wall if we have new data from
            the laser scanner right in front of us """
        if msg.ranges[45] > 0.0:
            self.front_pt  = msg.ranges[45]
            print("front", self.front_pt)
        if msg.ranges[135] > 0.0:
            self.back_pt  = msg.ranges[135]
            print("back", self.back_pt)

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
                self.angular_vel = 0
                self.linear_vel = .2
                if self.front_pt is not None and self.back_pt is not None:
                    self.state = "following"
            elif self.state == "following":
                if self.front_pt > self.back_pt + self.threshold:
                    self.angular_vel = 2*(self.front_pt-self.back_pt)
                    print("pos angular")
                elif self.back_pt > self.front_pt + self.threshold:
                    self.angular_vel = 2*(self.front_pt-self.back_pt)
                    print("neg angular")
                else:
                    # Going pretty stright
                    self.angular_vel = 0
                    pass


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

