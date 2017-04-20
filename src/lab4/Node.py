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
				time.sleep(.05)
				pos = Point()
				pos.x = self.coord.x + x
				pos.y = self.coord.y + y
				pos.z = self.coord.z

				newNode = Node(pos, self, self.dist + math.sqrt(x**2 + y**2))

				if self.contains(pos, res):
					pass
				else:
					myNeighbors.append(newNode)

		# print 'neighbors:', myNeighbors

		return myNeighbors

	def contains(self, pos, res):
		#print self.coord.x + res/2, self.coord.x - res/2, pos.x, self.coord.y + res/2, self.coord.y - res/2, pos.y
		if self.coord.x + res/2 > pos.x and self.coord.x - res/2 < pos.x and self.coord.y + res/2 > pos.y and self.coord.y - res/2 < pos.y:
			return True
		# elif not self.parent:
		# 	return False
		else:
			return False