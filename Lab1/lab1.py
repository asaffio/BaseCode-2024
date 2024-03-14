#!/usr/bin/env python3
import rospy
import toplevel


if __name__ == '__main__':
    rospy.init_node('airob_lab1', anonymous=False)
    toplevel = toplevel.TopLevelLoop(goal = (5.0, 1.0, ), debug = 1)
