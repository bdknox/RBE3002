#!/usr/bin/env python

import rospy, tf, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion

wheel_sep = 10
# Add additional imports for each of the message types used


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
	ang_vel = ((u1 - u2)/wheel_sep)
	now = rospy.Time.now().secs
	while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
		publishTwist(lin_vel, ang_vel)
	publishTwist(0, 0)




#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
	global pose
	curPose = pose

	atTarget = False
	while (not atTarget and not rospy.is_shutdown()):
		curDist = math.sqrt((curPose.position.x - pose.position.x)**2 + (curPose.position.y - pose.position.y)**2)
		print curDist
		rospy.sleep(rospy.Duration(.01, 0))
		if (curDist >= distance):
			print "Made it!"
			atTarget = True
			publishTwist(0,0)
		else:
			publishTwist(speed, 0)




    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
	global pub
	global pose
	global theta

	goalAng = theta - angle

	while not (angle - .05 <= theta <= angle + .05):
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




# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('bdknox_lab2_node')

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
    global pub
    global pose
    global odom_tf
    global odom_list
   
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=5) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds

    spinWheels(1, 1, 1)
    

    #make the robot keep doing something...
    rospy.Timer(rospy.Duration(.01), timerCallback)
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting Lab 2"

    driveStraight(5, 2)

    # Make the robot do stuff...
    print "Lab 2 complete!"

