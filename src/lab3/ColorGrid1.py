#!/usr/bin/env python
import rospy, numpy, math
from nav_msgs.msg import GridCells, OccupancyGrid
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped
from Queue import PriorityQueue
from math import sqrt
from astar_goodish import aStar

def publishColor(color):
	global pub
	pub.publish(color)

def setGridCells(msg):
	global cell_width
	global cell_height
	global instance
	global res

	cell_height = msg.info.height
	cell_width = msg.info.width
	instance = msg.data
	res = msg.info.resolution

def drawWalls(grid_pub):
	global height
	global width
	global obstacles_pub
	i = 0

	obstacles=GridCells()
	obstacles.header.frame_id = 'map'
	obstacles.cell_width = 0.3
	obstacles.cell_height = 0.3
	for y in range (0, height):
		for x in range (0, width):
			if (grid[i] == 100):
				obstacles.cells.append(Point(x*0.3+0.7, y*0.3+0.2, 0))

			i = i + 1
	obstacles_pub.publish(obstacles)

def mapCallBack(data):
	global width
	global height
	global grid

	width = data.info.width
	height = data.info.height
	grid = data.data

def startPoseCallback(pose):
	global start

	x = int(pose.pose.position.x)
	y = int(pose.pose.position.y)

	start = (x,y)

def endPoseCallback(pose):
	global goal

	x = int(pose.pose.position.x)
	y = int(pose.pose.position.y)

	goal = (x,y)

# main
if 	__name__ == "__main__":
	global cell_width
	global cell_height
	global instance
	global res
	global obstacles
	global unexplored
	global width
	global height
	global grid
	global obstacles_pub
	global start
	global goal

	rospy.init_node('Color')

	grid = list()
	start = (0,0)

	frontier_pub = rospy.Publisher('frontier', GridCells, queue_size=10)
	# explored_pub = rospy.Publisher('explored', GridCells, queue_size=10)
	# shortpath_pub  = rospy.Publisher('shortpath', GridCells, queue_size=10)
	obstacles_pub = rospy.Publisher('obstacles', GridCells, queue_size=10)
	unexplored_pub = rospy.Publisher('unexplored', GridCells, queue_size=10)

	initpose_sub = rospy.Subscriber('start_pose', PoseStamped, startPoseCallback, queue_size=10)
	finalpose_sub = rospy.Subscriber('goal_pose', PoseStamped, endPoseCallback, queue_size=10)
	occupancygrid_sub = rospy.Subscriber('map', OccupancyGrid, mapCallBack, queue_size=10)

while not rospy.is_shutdown():
	storeobs = []
	storeunex = []

	# frontier
	# frontier=GridCells()
	# frontier.header.frame_id = 'map'
	# frontier.cell_width = 0.3
	# frontier.cell_height = 0.3
	# frontier.cells = 
	# frontier.cells.append(Point(1,1,0))

	# explored
	# explored=GridCells()
	# explored.header.frame_id = 'map'
	# explored.cell_width = 0.3
	# explored.cell_height = 0.3
	# explored.cells.append(Point(1,2,0))

	# shortestpath
	# shortpath=GridCells()
	# shortpath.header.frame_id = 'map'
	# shortpath.cell_width = 0.3
	# shortpath.cell_height = 0.3
	# shortpath.cells.append(Point(1,3,0))

	# unexplored
	unexplored=GridCells()
	unexplored.header.frame_id = 'map'
	unexplored.cell_width = 0.3
	unexplored.cell_height = 0.3
	unexplored.cells = storeunex

	# publish
	# frontier_pub.publish(frontier)
	# frontier_pub.publish(explored)
	#frontier_pub.publish(obstacles)
	#frontier_pub.publish(unexplored)
	# frontier_pub.publish(shortpath)
	rospy.sleep(rospy.Duration(1))

	drawWalls(grid)

	try:
		goal
	except NameError:
		continue
	else:
		aStar(start, goal, grid)

