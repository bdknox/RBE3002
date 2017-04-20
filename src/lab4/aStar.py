import rospy, tf, numpy, math,time
import Node
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry, Path, OccupancyGrid, MapMetaData, GridCells
from geometry_msgs.msg import Twist, PoseStamped

class aStar(object):
    def __init__(self):

        self.resolution = .21
        self.robot = .21

        self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.createMap, queue_size=1)

        self.frontier_pub = rospy.Publisher('frontier', GridCells, queue_size=10)
        self.explored_pub = rospy.Publisher('explored', GridCells, queue_size=10)
        #shortpath_pub  = rospy.Publisher('shortpath', GridCells, queue_size=10)
        #obstacles_pub = rospy.Publisher('obstacles', GridCells, queue_size=10)
        #unexplored_pub = rospy.Publisher('unexplored', GridCells, queue_size=10)

    def justDoIt(self, startCoord, goalCoord):
        self.startCoord = startCoord
        startNode = Node.Node(startCoord, None, 0)
        self.frontier = []
        self.frontier.append(startNode)
        self.explored = []
        self.goalCoord = goalCoord
        print goalCoord

        self.pub = GridCells()
        self.pub.header = self.map.header
        self.pub.cell_width = self.map.info.resolution
        self.pub.cell_height = self.map.info.resolution

        self.path = GridCells()
        self.path.header = self.map.header
        self.path.cell_width = self.map.info.resolution
        self.path.cell_height = self.map.info.resolution

        while not self.frontier[0].contains(goalCoord, self.robot):
            next = self.frontier.pop(0)
            if self.wasHere(next):
                continue
            self.explored.append(next)
            if self.frontier:
                self.pub.cells.remove(next.coord)

            neighbors = next.getNeighbors(self.map.info.resolution)

            for node in neighbors:
                if self.isValid(node.coord):
                    if not self.wasHere(node) or self.wasFound(node):
                        if not self.frontier:
                            self.frontier.append(node)
                        else:
                            node_cost = node.dist + self.hCost(node.coord, self.goalCoord) #self.goal?
                            for i, j in enumerate(self.frontier):
                                j_cost = j.dist + self.hCost(j.coord, self.goalCoord)
                                if (node_cost < j_cost):
                                    self.frontier.insert(i, node)
                                    break
                                elif (i == len(self.frontier)-1):
                                    self.frontier.append(node)
                                    break

                        self.pub.cells.append(node.coord)

            self.frontier_pub.publish(self.pub)
        print 'suh dude where ya been?'

    def isValid(self, pos):
        room = self.robot + self.resolution/2
        # print pos.x
        for x in numpy.arange(pos.x - room, pos.x + room, self.map.info.resolution):
            for y in numpy.arange(pos.y - room, pos.y + room, self.map.info.resolution):
                if x > self.map.info.origin.position.x and y > self.map.info.origin.position.y:
                    xPoint = int((x - self.map.info.origin.position.x)/self.map.info.resolution)
                    yPoint = int((y - self.map.info.origin.position.y)/self.map.info.resolution)
                    # print 'point:', xPoint, yPoint
                    indx = xPoint + self.map.info.width*yPoint
                    # print indx
                    try:
                        if self.map.data[indx] is 100:
                            return True
                    except:
                        return False
        return True

    def wasHere(self, node):
        for ex in self.explored:
            if ex.contains(node.coord, self.resolution):
                return True
            return False

    def wasFound(self, node):
        for f in self.frontier:
            if f.contains(node.coord, self.resolution):
                return True
            return False

    def hCost(self, pos, goal):
        return math.sqrt((pos.x - goal.x)**2 + (pos.y - goal.y)**2 + (pos.z - goal.z)**2)

    def createMap(self, msg):
        self.map = msg

    def getPoints(self):
        pass