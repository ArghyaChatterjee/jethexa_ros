#!/usr/bin/env python3
# coding: utf-8

import sys
import rospy
import curses
from geometry_msgs.msg import Twist
from jethexa_controller_interfaces.msg import Traveling


def main(stdscr):
    msg = Twist()
    last_keycode = ord('.')
    curses.curs_set(0)
    stdscr.addstr(0, 0, "Enter : Stand         |  Space : Stop")
    stdscr.addstr(1, 0, "W / ↑ : Move Forward  |  S / ↓ : Move Backward")
    stdscr.addstr(2, 0, "A     : Move Left     |  D     : Move Right")
    stdscr.addstr(3, 0, "←     : Trun Left     |  →     : Turn Right")
    stdscr.addstr(5, 0, ">>>          <<<")
    while not rospy.is_shutdown():
        keycode = stdscr.getch()
        if keycode == ord('\n'):
            stdscr.addstr(1, 0, "Stand")
            tmsg = Traveling()
            tmsg.gait = -2
            tmsg.time = 1.0
            traveling_pub.publish(tmsg)
        if last_keycode == keycode:
            continue
        last_keycode = keycode

        if keycode == curses.KEY_UP or keycode == ord('w'):
            stdscr.addstr(5, 0, " "*40)
            stdscr.addstr(5, 0, ">>> Move Forward <<<")
            msg.linear.x, msg.linear.y = 0.08, 0.0
            msg.angular.z = 0.0
            cmd_vel_pub.publish(msg)

        elif keycode == curses.KEY_DOWN or keycode == ord('s'):
            stdscr.addstr(5, 0, " "*40)
            stdscr.addstr(5, 0, ">>> Move Backward <<<")
            msg.linear.x, msg.linear.y = -0.08, 0.0
            msg.angular.z = 0.0
            cmd_vel_pub.publish(msg)

        elif keycode == curses.KEY_LEFT:
            stdscr.addstr(5, 0, " "*40)
            stdscr.addstr(5, 0, "Turn Left")
            msg.linear.x, msg.linear.y = 0.0, 0.0
            msg.angular.z = 0.25
            cmd_vel_pub.publish(msg)

        elif keycode == curses.KEY_RIGHT:
            stdscr.addstr(5, 0, " "*40)
            stdscr.addstr(5, 0, ">>> Turn Right <<<")
            msg.linear.x, msg.linear.y = 0.0, 0.0
            msg.angular.z = -0.25
            cmd_vel_pub.publish(msg)

        elif keycode == ord('a'):
            stdscr.addstr(5, 0, " "*40)
            stdscr.addstr(5, 0, ">>> Move Left <<<")
            msg.linear.x, msg.linear.y = 0.0, 0.05
            msg.angular.z = 0.0
            cmd_vel_pub.publish(msg)

        elif keycode == ord('d'):
            stdscr.addstr(5, 0, " "*40)
            stdscr.addstr(5, 0, ">>> Move Right <<<")
            msg.linear.x, msg.linear.y = 0.0, -0.05
            msg.angular.z = 0.0
            cmd_vel_pub.publish(msg)

        elif keycode == ord(' '):
            stdscr.addstr(5, 0, " "*40)
            stdscr.addstr(5, 0, ">>> Stop <<<")
            msg.linear.x, msg.linear.y = 0.0, 0.0
            msg.angular.z = 0.0
            cmd_vel_pub.publish(msg)

        else:
            pass


if __name__ == '__main__':
    try:
        rospy.init_node('keyboard_cotnrol_node')
        topic_prefix = rospy.get_param("~topic_prefix", "jethexa_controller")
        cmd_vel_pub = rospy.Publisher(topic_prefix + '/cmd_vel', Twist, queue_size=1)
        traveling_pub = rospy.Publisher(topic_prefix + '/traveling', Traveling, queue_size=1)
        curses.wrapper(main)
    except Exception as e:
        rospy.logerr(str(e))

