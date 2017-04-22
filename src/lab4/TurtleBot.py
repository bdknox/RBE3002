import rospy, tf, numpy, math, time
import aStar
import geometry_msgs
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, Pose
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion

class TurtleBot(object):

	wheel = .23
	radius = .035

	max_linear = 5
	min_linear = .5

	max_angular = 2
	min_angular = .1

	stop_msg = Twist()
	stop_msg.linear.x = 0
	stop_msg.angular.z = 0

	def __init__(self):
		print 'creating Turtlebot'
		self.transfromation = tf.TransformerROS()

		self.pose = Pose()
		pos = Point()
		quarter = Quaternion()
		self.pose.position.x = 1.23
		self.pose.position.y = 0.92
		self.pose.position.z = 0
		self.path = []

		self.initPos = [0,0,0]
		self.initOrient = [0,0,0,0]

		self.hopeful = [0,0,0]

		self.isInit = False

		self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10)
		self.goal_sub = rospy.Subscriber('goal_pose', PoseStamped, self.executeAStar, queue_size=1)
		#self.bump_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.readBumper, queue_size=1)
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.detOdometry)
		self.odom_tfList = tf.TransformListener()
		self.odom_broad = tf.TransformBroadcaster()

		print 'Completed making Turtlebot'

	def executeAStar(self, msg):
		astart = aStar.aStar()
		time.sleep(.25)
		astart.justDoIt(self.pose.position, msg.pose.position)
		self.path = astart.getPath()
		print self.path
		astart.wayPoints()

	def detOdometry(self, msg):
		if not self.isInit:
			try:
				(position, orientation) = odom_tfList.lookupTransform('map','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)

				self.initPos = position
				self.initOrient = orientation

				roll, pitch, yaw = euler_from_quaternion(orientation)

				self.initTheta = math.degrees(yaw)

				self.isInit = True

			except:
				print 'Sorry ahso'


		else:
			(position, orientation) = odom_tfList.lookupTransform('map','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)

			self.pose.position.x = position[0]
			self.pose.position.y = position[1]
			self.pose.position.z = position[2]

			self.pose.orientation.x = orientation[0]
			self.pose.orientation.y = orientation[1]
			self.pose.orientation.z = orientation[2]
			self.pose.orientation.w = orientation[3]
			

			roll, pitch, yaw = euler_from_quaternion(orientation)
			self.theta = math.degrees(yaw)


	#drive to a goal subscribed as /move_base_simple/goal
	def navToPose(goal):
	 #    print "spin!"
	 #    print "move!"
	 #    driveStraight()
	    # print "spin!"
	    # spinWheels()
	    # print "done"
	    pass


	#This function sequentially calls methods to perform a trajectory.
	def executeTrajectory():
		global theta

		driveStraight(-5, .6)
		rotate(theta + 90)
		driveStraight(5, .45)
		rotate(theta + 135)
		

	#This function accepts two wheel velocities and a time interval.
	def spinWheels(u1, u2, time):
		global pub
		lin_vel = ((u1 + u2)/2)
		ang_vel = ((u1 - u2)/wheel)
		now = rospy.Time.now().secs
		while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
			publishTwist(lin_vel, ang_vel)
		publishTwist(0, 0)


	#This function accepts a speed and a distance for the robot to move in a straight line
	def driveStraight(self, speed, distance):
		curPose = self.pose

		atTarget = False
		while (not atTarget and not rospy.is_shutdown()):
			curDist = math.sqrt((curPose.position.x - self.pose.position.x)**2 + (curPose.position.y - self.pose.position.y)**2)
			scaler = .5/(abs(distance/2 - curDist) + .1)
			print curDist
			rospy.sleep(rospy.Duration(.01, 0))
			if (curDist >= distance):
				atTarget = True
				publishTwist(0,0)
			else:
				publishTwist(scaler*speed, 0)

	def stopRobot(self):
		self.pub.publish(Robot.stop_msg)
		return True

	#Accepts an angle and makes the robot rotate around it.
	def rotate(self, angle):

		goalAng = self.theta + angle

		while not (goalAngle - .05 <= self.theta <= goalAngle + .05):
			diff = abs(theta - angle)
			if goalAng >= 0:
				publishTwist(0, -.01*diff)
			elif goalAng < 0:
				publishTwist(0, .01*diff)
			print theta
		publishTwist(0, 0)


	#This function works the same as rotate how ever it does not publish linear velocities.
	def driveArc(radius, speed, angle):
	    pass  # Delete this 'pass' once implemented

	    #Use rotate to driveArc in radius

	#Bumper Event Callback function
	def readBumper(msg):
	    if (msg.state == 1):
	        print "Bumper pressed!"
	        executeTrajectory()

	# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
	# Start the timer with the following line of code: 
	#   rospy.Timer(rospy.Duration(.01), timerCallback)
	def timerCallback(event):
		global pose
		pose = Pose()
		global theta

		(position, orientation) = odom_list.lookupTransform('/odom','/base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
		pose.position.x = position[0]
		pose.position.y = position[1]
		odomW = orientation
		q = [odomW[0], odomW[1], odomW[2], odomW[3]]
		roll, pitch, yaw = euler_from_quaternion(q)
		pose.orientation.z = yaw
		theta = math.degrees(yaw)

	def publishTwist(linearVelocity, angularVelocity):
	    # Send a movement (twist) message.
	    global pub
	    msg = Twist()
	    msg.linear.x = linearVelocity
	    msg.angular.z = angularVelocity
	    pub.publish(msg)
