#!/usr/bin/env python
import rospy, aStar, TurtleBot

from geometry_msgs.msg import Twist, PoseStamped
rospy.init_node('lab4')

robot = TurtleBot.TurtleBot()

rospy.spin()