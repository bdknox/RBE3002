import math, geometry_msgs, time

from geometry_msgs.msg import Point

class Node(object):
	def __init__(self, coord, parent, dist):
		self.coord = coord
		self.parent = parent
		self.dist = dist

	def getNeighbors(self, res):
		myNeighbors =[]

		for x in [-res,0,res]:
			for y in [-res,0,res]:
				pos = Point()
				pos.x = self.coord.x + x
				pos.y = self.coord.y + y
				pos.z = self.coord.z

				newNode = Node(pos, self, self.dist + math.sqrt(x**2 + y**2))

				if self.contains(pos):
					pass
				else:
					myNeighbors.append(newNode)

	def contains(self, pos):
		return (abs(self.coord.x - pos.x) + abs(self.coord.y - pos.y) + abs(self.coord.z - pos.z) < .01)