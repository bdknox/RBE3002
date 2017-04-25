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
        self.transform = tf.TransformerROS()

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
        #self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odometryCallback)
        self.odom_tfList = tf.TransformListener()
        self.odom_broad = tf.TransformBroadcaster()
        rospy.Timer(rospy.Duration(.01), self.odometryCallback)

        print 'Completed making Turtlebot'

    def executeAStar(self, msg):
        astart = aStar.aStar()
        time.sleep(.25)
        astart.justDoIt(self.pose.position, msg.pose.position)
        self.path = astart.getPath()
        print self.path
        wayPoints = astart.wayPoints()

        print wayPoints

        for pnt in wayPoints:
            nPose = Pose()
            nPose.position = pnt
            nPose.orientation = self.pose.orientation
            self.navToPose(nPose)
        print 'lol gotem who needs to actually drive amirite'

    def odometryCallback(self, event):
        if not self.isInit:
            try:
                (position, orientation) = self.odom_tfList.lookupTransform('/map','/base_footprint', rospy.Time(0))

                self.initPos = position
                self.initOrient = orientation

                roll, pitch, yaw = euler_from_quaternion(orientation)

                self.initTheta = math.degrees(yaw)

                self.isInit = True

                print self.initTheta, self.initPos

            except:
                pass
                #print 'Sorry ahso'


        else:
            (position, orientation) = self.odom_tfList.lookupTransform('/map','/base_footprint', rospy.Time(0)) 

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
    def navToPose(self, goal):
        self.getDesiredPose(goal)
        self.rotate()
        self.driveStraight(.01)


    def getDesiredPose(self, goal):
        self.hopeful[0] = goal.position.x
        self.hopeful[1] = goal.position.y
        quarter = goal.orientation
        quarter = [quarter.x, quarter.y, quarter.z, quarter.w]
        roll, pitch, yaw = euler_from_quaternion(quarter)
        theta = math.degrees(yaw)
        if (self.hopeful[0] == self.pose.position.x and self.hopeful[1] == self.pose.position.y):
            print 'looks good'
            self.hopeful[2] = theta
        else:
            print 'setting new value'
            self.hopeful[2] = math.degrees(math.atan2((self.hopeful[1] - self.pose.position.y), (self.hopeful[0] - self.pose.position.x)))
        print self.hopeful[2]

    #Accepts an angle and makes the robot rotate around it.
    def rotate(self):
        goalAng = self.hopeful[2]
        print goalAng
        print self.theta
        while not (goalAng - .05 <= self.theta <= goalAng + .05):
            diff = abs(self.theta - goalAng)
            if goalAng >= 0:
                self.moveTurtle(0, -.01*diff)
                #self.moveTurtle(0, -.5)
            elif goalAng < 0:
                self.moveTurtle(0, .01*diff)
                #self.moveTurtle(0, 1)
            #rospy.sleep(rospy.Duration(.01, 0))
        print self.theta, goalAng
        self.stopRobot()

    #This function accepts a speed and a distance for the robot to move in a straight line
    def driveStraight(self, speed):
        # curPose = self.pose
        # atTarget = False
        distance = math.sqrt((self.hopeful[0] - self.pose.position.x)**2 + (self.hopeful[1] - self.pose.position.y)**2)
        print distance
        while distance > .1:
            scaler = 1/(abs(distance/2 - distance) + .1)
            self.moveTurtle(scaler*speed, 0)
            distance = math.sqrt((self.hopeful[0] - self.pose.position.x)**2 + (self.hopeful[1] - self.pose.position.y)**2)
            print distance
            #rospy.sleep(rospy.Duration(.01, 0))
        print 'drove straight hahahaahahahahahhahhahha jk but no really tecnically i did'
        self.stopRobot()

        # testing out this new way to complete drive straight, commented out lab2 way

        # while (not atTarget and not rospy.is_shutdown()):
        #   curDist = math.sqrt((curPose.position.x - self.pose.position.x)**2 + (curPose.position.y - self.pose.position.y)**2)
        #   scaler = .5/(abs(distance/2 - curDist) + .1)
        #   print curDist
        #   rospy.sleep(rospy.Duration(.01, 0))
        #   if (curDist >= distance):
        #       atTarget = True
        #       moveTurtle(0,0)
        #   else:
        #       moveTurtle(scaler*speed, 0)


    #This function sequentially calls methods to perform a trajectory.
    # def executeTrajectory():
    #   global theta

    #   driveStraight(-5, .6)
    #   rotate(theta + 90)
    #   driveStraight(5, .45)
    #   rotate(theta + 135)
        

    #This function accepts two wheel velocities and a time interval.
    # def spinWheels(u1, u2, time):
    #   global pub
    #   lin_vel = ((u1 + u2)/2)
    #   ang_vel = ((u1 - u2)/wheel)
    #   now = rospy.Time.now().secs
    #   while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
    #       moveTurtle(lin_vel, ang_vel)
    #   moveTurtle(0, 0)

    def stopRobot(self):
        self.pub.publish(TurtleBot.stop_msg)
        return True


    #This function works the same as rotate how ever it does not publish linear velocities.
    def driveArc(radius, speed, angle):
        pass  # Delete this 'pass' once implemented

        #Use rotate to driveArc in radius

    #Bumper Event Callback function
    def readBumper(msg):
        if (msg.state == 1):
            print "Bumper pressed!"
            self.executeTrajectory()

    # (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
    # Start the timer with the following line of code: 
    #   rospy.Timer(rospy.Duration(.01), timerCallback)
    def timerCallback(event):
        self.odometryCallback()

    def moveTurtle(self, linearVelocity, angularVelocity):
        msg = Twist()
        msg.linear.x = linearVelocity
        msg.angular.z = angularVelocity

        if angularVelocity > 2:
            msg.angular.z = 2
        elif angularVelocity < -2:
            msg.angular.z = -2

        if linearVelocity > 5:
            msg.linear.x = 5
        elif linearVelocity < -5:
            msg.linear.x = -5
        self.pub.publish(msg)