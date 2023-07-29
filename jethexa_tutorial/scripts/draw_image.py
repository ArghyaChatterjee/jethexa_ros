#!/usr/bin/env python3
import rospy
import math
from jethexa_controller import client



def draw_triangle(jethexa):
    for i in range(2):
        jethexa.set_leg_relatively(1, 80.0, -40.0, 160.0, 0.8)
        rospy.sleep(0.8)
        jethexa.set_leg_relatively(1, 80.0, 0.0, 80.0, 0.8)
        rospy.sleep(0.8)
        jethexa.set_leg_relatively(1, 80.0, -80.0, 80.0, 0.8)
        rospy.sleep(0.8)

def draw_square(jethexa):
    for i in range(2):
        jethexa.set_leg_relatively(1, 80.0, -80.0, 160.0, 0.8)
        rospy.sleep(0.8)
        jethexa.set_leg_relatively(1, 80.0, 0.0, 160.0, 0.8)
        rospy.sleep(0.8)
        jethexa.set_leg_relatively(1, 80.0, 0.0, 80.0, 0.8)
        rospy.sleep(0.8)
        jethexa.set_leg_relatively(1, 80.0, -80.0, 80.0, 0.8)
        rospy.sleep(0.8)


def draw_star(jethexa):
    jethexa.set_leg_relatively(1, 80.0, -80.0, 80.0, 0.8)
    rospy.sleep(0.8)
    for i in range(2):
        jethexa.set_leg_relatively(1, 80.0, -30.0, 170.0, 0.8)
        rospy.sleep(0.8)
        jethexa.set_leg_relatively(1, 80.0, 10.0, 80.0, 0.8)
        rospy.sleep(0.8)
        jethexa.set_leg_relatively(1, 80.0, -80.0, 130.0, 0.8)
        rospy.sleep(0.8)
        jethexa.set_leg_relatively(1, 80.0, 10.0, 130.0, 0.8)
        rospy.sleep(0.8)
        jethexa.set_leg_relatively(1, 80.0, -80.0, 80.0, 0.8)
        rospy.sleep(0.8)

def draw_circle(jethexa):
    jethexa.set_leg_relatively(1, 80.0, -80.0, 80.0, 0.8)
    rospy.sleep(0.8)
    points = []
    for i in range(180, 0, -3):
        x = 40 * math.cos(math.radians(i))
        y = 40 * math.sin(math.radians(i))
        points.append((x, y))
    for i in range(360, 180, -3):
        x = 40 * math.cos(math.radians(i))
        y = 40 * math.sin(math.radians(i))
        points.append((x, y))
    print(points)
    for x, y in points:
        jethexa.set_leg_relatively(1, 80.0, -40.0 + x, 80.0 + y, 0.01)
        rospy.sleep(0.01)


def init(jethexa):
    jethexa.set_leg_relatively(1, 80.0, -80.0, 80.0, 0.8)
    rospy.sleep(1.2)


if __name__ == "__main__":
    rospy.init_node('adfas')
    jethexa = client.Client(None)
    rospy.sleep(0.5)
    init(jethexa)
    draw_circle(jethexa)
