#!/usr/bin/env python
import rospy, numpy, math
from nav_msgs.msg import GridCells, OccupancyGrid
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped
from Queue import PriorityQueue
from math import sqrt


def genGC(tupel):
    newCell = GridCell(tupel[0], tupel[1], 0, 0, False)
    newCell.gcost = manhattan(tupel,start)
    newCell.hcost = manhattan(tupel,goal)
    if grid[tupel[0]+tupel[1]*37] == 100:
        newCell.isWall = True
    return newCell


# Calculating the remaining distance, hcost determined by the distance between the projection of 
# two points = |x1-x2| + |y1-y2|
def manhattan(cur,fin):
    # print cur, fin
    return abs(cur[0] - fin[0]) + abs(cur[1] - fin[1])
    
    #return abs(point.point[0] - point2.point[0]) + abs(point.point[1]-point2.point[0])

def convertLocation(location):
    global res
    global grid
    global origin
    print res 
    x = (location[0] + origin.position.x)/res
    y = (location[1] - origin.position.y)/res
    return (int(x),int(y))


def aStar(startCell, goalCell, grid):
    #The open and closed sets
    # print ourMap[-1][1].x, ourMap[-1][1].y
    start = convertLocation(startCell)
    print start[0]
    print start[1]
    goal = convertLocation(goalCell)
    open_set = set()
    closed_set = set()
    current = genGC(start)
    #Add the starting point to the open set
    open_set.add(current)
    print open_set
    #While the open set is not empty
    while open_set:
        drawWalls(open_set)
        rospy.sleep(rospy.Duration(0.05))
        #Find the item in the open set with the lowest G + H score
        #current = min(open_set, key=lambda o:o.gcost + o.hcost)
        #If it is the item we want, retrace the path and return it
        if current == goal:
            path = []
            # for a in closed_set:
            #   print a.parent.x, a.parent.y
            # print current.parent, current.parent.y
            while hasattr(current, 'parent'):#current.parent:
                path.append(current)
                current = current.parent
            path.append(current)
            return path[::-1]
        #Remove the item from the open set
        #print open_set.has_key(current.x + 37*current.y)
        print 'current:', current.x, current.y
        open_set.remove(current)
        print 'open set:', open_set
        #Add it to the closed set
        closed_set.add(current)
        print closed_set
        #Loop through the node's children/siblings
        for node in neighbors(current):
            if node is None:
                continue
            #If it is already in the closed set, skip it
            if node in closed_set:
                continue
            if node.isWall:
                continue
            #Otherwise if it is already in the open set
            if node in open_set:
                #Check if we beat the G score 
                new_g = current.gcost + 1 #+ current.move_cost(node)                         will need to change this
                if node.gcost > new_g:
                    #If so, update the node to have a new parent
                    node.gcost = new_g
                    node.parent = current
            else:
                #If it isn't in the open set, calculate the G and H score for the node
                #node.gcost = current.gcost #+ current.move_cost(node)
                #node.hcost = manhattan(node, goal)
                #Set the parent to our current item
                node.parent = current
                # print node.parent.x, node.parent.y
                #Add it to the set
                open_set.add(node)
                if len(open_set) == 0:
                    current = node
                    print 'setting current'
                else:
                    curF = node.gcost + node.hcost
                    if curF < (current.gcost + current.hcost):
                        current = node
                        print 'setting current'
        #drawWalls(open_set)
    #Throw an exception if there is no path
    raise ValueError('No Path Found')

def neighbors(currentGC):
    x = currentGC.x
    y = currentGC.y
    index = x+(y*37)
    # print 'Calculating'
    # print x, y

    myNeighbors = list()

    if y < 36:
        upCoord = (x, y+1)
        print 'upCoord', upCoord[0], upCoord[1]
        upGC = genGC(upCoord)
        myNeighbors.append(upGC)
    if y > 0:
        downCoord = (x, y-1)
        downGC = genGC(downCoord)
        myNeighbors.append(downGC)
    if x < 36:
        rightCoord = (x+1, y)
        rightGC = genGC(rightCoord)
        myNeighbors.append(rightGC)
    if x > 0:
        leftCoord = (x-1, y)
        leftGC = genGC(leftCoord)
        myNeighbors.append(leftGC)

    return myNeighbors

def neighborsDiag(current, grid):

    myNeighbors = list()
    for x in range(-1, 2):
        for y in range(-1,2):
            notSelf = not(x==0 and y ==0)
            notGreater = (x<= width and y <= height)
            notnegative = (x>=0 and y>=0)
            if notSelf and notGreater and notnegative:
                myNeighbors.append(grid[current.x + x][current.y + y])

    return myNeighbors

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
    i = 0

    frontier_pub = rospy.Publisher('frontier', GridCells, queue_size=10)


    obstacles=GridCells()
    obstacles.header.frame_id = 'map'
    obstacles.cell_width = 0.3
    obstacles.cell_height = 0.3
    for gc in grid_pub:
        obstacles.cells.append(Point(gc.x*res+origin.position.x, gc.y*res+origin.position.y, 0))

        i = i + 1
    frontier_pub.publish(obstacles)

class GridCell:
    
    def __init__(self, x, y, gcost, hcost, isWall, parent = None):
        self.x = x
        self.y = y
        self.gcost = gcost
        self.hcost = hcost
        self.isWall = False
        self.parent = parent

def mapCallBack(data):
    global width
    global height
    global grid
    global origin
    global res
    print "this is mapCallBack"

    width = data.info.width
    height = data.info.height
    grid = data.data
    origin = data.info.origin
    res = data.info.resolution

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
if  __name__ == "__main__":
    # global cell_width
    # global cell_height
    # global instance
    # global res
    # global obstacles
    # global unexplored
    # global width
    # global height
    # global grid
    # global obstacles_pub
    # global start
    # global goal

    rospy.init_node('Color')

    grid = list()
    start = (1,1)
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

    rospy.sleep(rospy.Duration(1))

    try:
        goal
    except NameError:
        continue
    else:
        drawWalls(aStar(start, goal, grid))