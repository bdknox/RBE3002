import rospy, tf, numpy, math,time
import Node
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry, Path, OccupancyGrid, MapMetaData, GridCells
from geometry_msgs.msg import Twist, PoseStamped

class aStar(object):
    def __init__(self):

        self.resolution = .21
        self.robot = .21

        self.points = []

        self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.createMap, queue_size=1)

        self.frontier_pub = rospy.Publisher('frontier', GridCells, queue_size=10)
        self.explored_pub = rospy.Publisher('explored', GridCells, queue_size=10)
        self.shortpath_pub  = rospy.Publisher('shortpath', GridCells, queue_size=10)
        self.obstacles_pub = rospy.Publisher('obstacles', GridCells, queue_size=10)
        #unexplored_pub = rospy.Publisher('unexplored', GridCells, queue_size=10)

    def justDoIt(self, startCoord, goalCoord):
        self.startCoord = startCoord
        self.startNode = Node.Node(startCoord, None, 0)
        self.frontier = []
        self.frontier.append(self.startNode)
        self.explored = []
        self.goalCoord = goalCoord
        print goalCoord

        self.pub = GridCells()
        self.pub.header = self.map.header
        self.pub.cell_width = self.map.info.resolution
        self.pub.cell_height = self.map.info.resolution

        self.exp = GridCells()
        self.exp.header = self.map.header
        self.exp.cell_width = self.map.info.resolution
        self.exp.cell_height = self.map.info.resolution

        self.path = GridCells()
        self.path.header = self.map.header
        self.path.cell_width = self.map.info.resolution
        self.path.cell_height = self.map.info.resolution

        self.obs = GridCells()
        self.obs.header = self.map.header
        self.obs.cell_width = self.map.info.resolution
        self.obs.cell_height = self.map.info.resolution

        self.shortpath_pub.publish(self.path)
        self.pub.cells.append(self.startCoord)
        self.frontier_pub.publish(self.pub)

        while not self.frontier[0].contains(goalCoord, self.map.info.resolution):
            next = self.frontier.pop(0)
            self.pub.cells.remove(next.coord)
            if self.wasHere(next.coord):
                continue
            self.explored.append(next.coord)
            self.exp.cells.append(next.coord)

            neighbors = next.getNeighbors(self.map.info.resolution, self.startNode)

            for node in neighbors:
                if self.isValid(node.coord):
                    if not self.wasHere(node.coord):
                        if (len(self.frontier) == 0):
                            self.frontier.append(node)
                        else:
                            node_cost = node.dist + self.hCost(node.coord, self.goalCoord) #self.goal?
                            for i, j in enumerate(self.frontier):
                                j_cost = j.dist + self.hCost(j.coord, self.goalCoord)
                                if ((node_cost - .15 < j_cost) and ((node_cost - node.dist) < (j_cost - j.dist))):
                                    self.frontier.insert(i, node)
                                    break
                                elif (i is len(self.frontier)-1):
                                    self.frontier.append(node)
                                    break

                        self.pub.cells.append(node.coord)

            self.frontier_pub.publish(self.pub)
            self.explored_pub.publish(self.exp)
            self.obstacles_pub.publish(self.obs)
        self.frontier_pub.publish(self.pub)
        self.explored_pub.publish(self.exp)
        time.sleep(.5)
        print 'frontier:', len(self.pub.cells)
        print 'explored:', len(self.exp.cells)
        print 'suh dude where ya been?'
        

    def isValid(self, pos):
        room = self.robot + self.map.info.resolution/2
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
                            self.obs.cells.append(pos)
                            return False
                    except:
                        return False
        return True

    def wasHere(self, pos):
        for ex in self.explored:
            if pos.x >= ex.x and pos.x < (ex.x + self.resolution) and pos.y >= ex.y and pos.y < (ex.y + self.resolution):
                return True
        return False

    def wasFound(self, pos):
        for fro in self.frontier:
            if pos.x >= fro.coord.x and pos.x < (fro.coord.x + self.resolution) and pos.y >= fro.coord.y and pos.y < (fro.coord.y + self.resolution):
                return True
        return False

    def hCost(self, pos, goal):
        return math.sqrt((pos.x - goal.x)**2 + (pos.y - goal.y)**2 + (pos.z - goal.z)**2)

    def createMap(self, msg):
        self.map = msg

    def getPath(self):
        cell = self.frontier[0]
        self.exp.cells.append(cell.coord)
        self.explored_pub.publish(self.exp)
        # print self.exp.cells
        while cell is not self.startNode:
            time.sleep(.1)
            # print cell.coord.x, cell.coord.y
            self.path.cells.insert(0, cell.coord)
            if self.wasHere(cell.coord):
                self.exp.cells.remove(cell.coord)
            cell = cell.parent
            self.shortpath_pub.publish(self.path)
            self.explored_pub.publish(self.exp)
        time.sleep(.1)
        print self.startCoord.x, self.startCoord.y
        self.path.cells.insert(0, self.startCoord)
        if self.wasHere(self.startCoord):
                self.exp.cells.remove(self.startCoord)
        self.shortpath_pub.publish(self.path)
        self.explored_pub.publish(self.exp)
        # print self.path.cells
        # print 'printed path'

        return self.path.cells

    def wayPoints(self):
        for idx, cell in enumerate(self.path.cells):
            if idx < (len(self.path.cells) - 1):
                nextCell = self.path.cells[idx + 1]
                newTheta = math.degrees(math.atan2((nextCell.y - cell.y),(nextCell.x - cell.x)))
                if len(self.points) == 0:
                    self.points.append(cell)
                    curTheta = newTheta
                    print 'Start oriented at', curTheta, 'degrees'
                elif abs(newTheta - curTheta) > 3:
                    self.points.append(cell)
                    lastPoint = self.points[len(self.points) - 2]
                    distFor = math.sqrt((cell.x - lastPoint.x)**2 + (cell.y - lastPoint.y)**2)
                    print 'Drive Forward', distFor, 'meters'
                    if abs(newTheta - curTheta) < 180:
                        print 'At Point:', cell.x, cell.y, 'Turn',  (newTheta - curTheta), 'degrees'
                    else:
                        print 'At Point:', cell.x, cell.y, 'Turn',  (newTheta + curTheta), 'degrees'
                    curTheta = newTheta
            else:
                self.points.append(cell)
                lastPoint = self.points[len(self.points) - 2]
                distFor = math.sqrt((cell.x - lastPoint.x)**2 + (cell.y - lastPoint.y)**2)
                print 'Drive Forward', distFor, 'meters, congratulations you did it my man'
