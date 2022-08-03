#!/usr/bin/env python3

from __future__ import print_function

import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist

msg = """
---------------------------
Moving around:
        w
    a   s   d

anything else : stop

CTRL-C to quit
---------------------------
"""

moveBindings = {
    'w': (1, 0, 0, 0),
    'a': (0, 0, 0, 1),
    'd': (0, 0, 0, -1),
    's': (-1, 0, 0, 0)
}


class Vel_Publisher:

    def __init__(self, rate):

        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 20.0
        self.turn = 10.0

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

    def wait_for_subscribers(self):

        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                i = -1
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1

        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th):

        self.x = x
        self.y = y
        self.z = z
        self.th = th

    def stop(self):
        self.update(0, 0, 0, 0)

    def run(self):

        twist = Twist()
        twist.linear.x = self.x * self.speed
        twist.linear.y = self.y * self.speed
        twist.linear.z = self.z * self.speed
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.th * self.turn

        # Publish.
        self.publisher.publish(twist)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == "__main__":

    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('teleop')

    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.1)

    publisher = Vel_Publisher(repeat)

    try:
        publisher.wait_for_subscribers()
        publisher.update(0, 0, 0, 0)
        print(msg)

        while True:
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]

            else:
                # Feed all zeroes to cmd_vel and robot should stop.
                x = 0
                y = 0
                z = 0
                th = 0
                if key == '\x03':
                    break

            publisher.update(x, y, z, th)
            publisher.run()

    except Exception as e:
        print(e)

    finally:
        publisher.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
