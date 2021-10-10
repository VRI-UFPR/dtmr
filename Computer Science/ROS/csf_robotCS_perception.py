#!/usr/bin/python2.7 
#DIGITAL TWIN MOBILE ROBOT
import rospy
import time
import math
import os

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

roll = pitch = yaw = yawDegree = 0.0

sonar1 = sonar2 = sonar3 = sonar4 = 0
sonar5 = sonar6 = sonar7 = sonar8 = 0

robot_x = robot_y = robot_z = 0.0
ac1_x = ac1_y = 0.0
ac2_x = ac2_y = 0.0
ac3_x = ac3_y = 0.0
ac4_x = ac4_y = 0.0
acX_z = 0.0

vetor_fingerMap=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
vetor_sonar=[0, 0, 0, 0, 0, 0, 0, 0]
vetor_sonarN=[0, 0, 0, 0, 0, 0, 0, 0]

gridMap = 1

def get_rotation (msg):
	global roll, pitch, yaw, yawDegree
	quaternion = (
		msg.x,	
		msg.y,
		msg.z,
		msg.w)
	euler = euler_from_quaternion (quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	#yawDegree = yaw	
	yawDegree = (((yaw*57.3))+180)	


def get_position_robot (msg_robot):
	global robot_x, robot_y
	robot_x = msg_robot.x
	robot_y = msg_robot.y
def get_position_ac1 (msg_ac1):
	global ac1_x, ac1_y
	ac1_x = msg_ac1.x
	ac1_y = msg_ac1.y
def get_position_ac2 (msg_ac2):
	global ac2_x, ac2_y
	ac2_x = msg_ac2.x
	ac2_y = msg_ac2.y	
def get_position_ac3 (msg_ac3):
	global ac3_x, ac3_y
	ac3_x = msg_ac3.x
	ac3_y = msg_ac3.y	
def get_position_ac4 (msg_ac4):
	global ac4_x, ac4_y
	ac4_x = msg_ac4.x
	ac4_y = msg_ac4.y

def euclidean_distance_ac1 ():
	global euclideanDistance1	
	acessPoint1 = (ac1_x, ac1_y, acX_z)
	robot = (robot_x, robot_y, robot_z)
	euclideanDistance1 = math.sqrt(sum([(a - b) ** 2 for a, b in zip(acessPoint1, robot)]))
	rospy.loginfo("Distance AP1 to Robot: %3.4f", euclideanDistance1)
def euclidean_distance_ac2 ():
	global euclideanDistance2	
	acessPoint2 = (ac2_x, ac2_y, acX_z)
	robot = (robot_x, robot_y, robot_z)
	euclideanDistance2 = math.sqrt(sum([(a - b) ** 2 for a, b in zip(acessPoint2, robot)]))
	rospy.loginfo("Distance AP2 to Robot: %3.4f", euclideanDistance2)	
def euclidean_distance_ac3 ():
	global euclideanDistance3
	acessPoint3 = (ac3_x, ac3_y, acX_z)
	robot = (robot_x, robot_y, robot_z)
	euclideanDistance3 = math.sqrt(sum([(a - b) ** 2 for a, b in zip(acessPoint3, robot)]))
	rospy.loginfo("Distance AP3 to Robot: %3.4f", euclideanDistance3)
def euclidean_distance_ac4 ():
	global euclideanDistance4
	acessPoint4 = (ac4_x, ac4_y, acX_z)
	robot = (robot_x, robot_y, robot_z)
	euclideanDistance4 = math.sqrt(sum([(a - b) ** 2 for a, b in zip(acessPoint4, robot)]))
	rospy.loginfo("Distance AP4 to Robot: %3.4f", euclideanDistance4)

def callback_s1(u1):
	global sonar1
	sonar1 = u1.data
	#rospy.loginfo("Sonar1: %.3f", u1.data)

def callback_s2(u2):
	global sonar2
	sonar2 = u2.data	

def callback_s3(u3):
	global sonar3
	sonar3 = u3.data	

def callback_s4(u4):
	global sonar4
	sonar4 = u4.data	

def callback_s5(u5):
	global sonar5
	sonar5 = u5.data	

def callback_s6(u6):
	global sonar6
	sonar6 = u6.data	

def callback_s7(u7):
	global sonar7
	sonar7 = u7.data

def callback_s8(u8):
	global sonar8
	sonar8 = u8.data	

rospy.init_node("csf_robotCS_perception")

sub_robotPosition = rospy.Subscriber('/positionRobot', Point, get_position_robot)
sub_robotYaw = rospy.Subscriber('/orientationRobot', Quaternion, get_rotation)

sub_ac1Position = rospy.Subscriber('/poseAP1', Point, get_position_ac1)
sub_ac2Position = rospy.Subscriber('/poseAP2', Point, get_position_ac2)
sub_ac3Position = rospy.Subscriber('/poseAP3', Point, get_position_ac3)
sub_ac4Position = rospy.Subscriber('/poseAP4', Point, get_position_ac4)

sub_sonar1 = rospy.Subscriber('/sonar1', Float32, callback_s1)
sub_sonar2 = rospy.Subscriber('/sonar2', Float32, callback_s2)
sub_sonar3 = rospy.Subscriber('/sonar3', Float32, callback_s3)
sub_sonar4 = rospy.Subscriber('/sonar4', Float32, callback_s4)
sub_sonar5 = rospy.Subscriber('/sonar5', Float32, callback_s5)
sub_sonar6 = rospy.Subscriber('/sonar6', Float32, callback_s6)
sub_sonar7 = rospy.Subscriber('/sonar7', Float32, callback_s7)
sub_sonar8 = rospy.Subscriber('/sonar8', Float32, callback_s8)

pub_sonarN = rospy.Publisher('/sonarN', Float32)
pub_sonarNE = rospy.Publisher('/sonarNE', Float32)
pub_sonarL = rospy.Publisher('/sonarL', Float32)
pub_sonarSE = rospy.Publisher('/sonarSE', Float32)
pub_sonarS = rospy.Publisher('/sonarS', Float32)
pub_sonarSO = rospy.Publisher('/sonarSO', Float32)
pub_sonarO = rospy.Publisher('/sonarO', Float32)
pub_sonarNO = rospy.Publisher('/sonarNO', Float32)

pub_AP1toRobot = rospy.Publisher('/AP1toRobot', Float32)
pub_AP2toRobot = rospy.Publisher('/AP2toRobot', Float32)
pub_AP3toRobot = rospy.Publisher('/AP3toRobot', Float32)
pub_AP4toRobot = rospy.Publisher('/AP4toRobot', Float32)

pub_robotOrientation = rospy.Publisher('/robotOrientation', Float32)

r = rospy.Rate(2)

while not rospy.is_shutdown():

	print ""	
	rospy.loginfo("***CSF - robotCS - Perception***")

	#teclado = raw_input('Read sensor data (1) Close read sensors (close): ')

	#vetor_sensores = vetor_sonarN[0]
	
	#move ()
	euclidean_distance_ac1 ()
	euclidean_distance_ac2 ()
	euclidean_distance_ac3 ()
	euclidean_distance_ac4 ()

	
	#rospy.loginfo("msg w: %.5f", msg.w)	

	rospy.loginfo("Robot Orientation (yaw): %.1f", yawDegree)	
	
	#Indica o sonar que esta apontando para o norte  
	if	yawDegree > 337.5 or yawDegree <= 22.5:
		norte = 0
		rospy.loginfo("Sonar 1 -> N")
	elif	yawDegree > 22.5 and yawDegree <= 67.5:
		norte = 1
		rospy.loginfo("Sonar 2 -> N")
	elif	yawDegree > 67.5 and yawDegree <= 112.5:
		norte = 2
		rospy.loginfo("Sonar 3 -> N")
	elif 	yawDegree > 112.5 and yawDegree <= 157.5:
		norte = 3
		rospy.loginfo("Sonar 4 -> N")
	elif	yawDegree > 157.5 and yawDegree <= 202.5:
		norte = 4
		rospy.loginfo("Sonar 5 -> N")
	elif	yawDegree > 202.5 and yawDegree <= 247.5:
		norte = 5
		rospy.loginfo("Sonar 6 -> N")
	elif	yawDegree > 247.5 and yawDegree <= 292.5:
		norte = 6
		rospy.loginfo("Sonar 7 -> N")
	elif	yawDegree > 292.5 and yawDegree <= 337.5:
		norte = 7
		rospy.loginfo("Sonar 8 -> N")

	
	rospy.loginfo("Sonar1: %.4f", sonar1)
	rospy.loginfo("Sonar2: %.4f", sonar2)
	rospy.loginfo("Sonar3: %.4f", sonar3)
	rospy.loginfo("Sonar4: %.4f", sonar4)	
	rospy.loginfo("Sonar5: %.4f", sonar5)
	rospy.loginfo("Sonar6: %.4f", sonar6)
	rospy.loginfo("Sonar7: %.4f", sonar7)
	rospy.loginfo("Sonar8: %.4f", sonar8)

	vetor_sonar = [sonar1, sonar2, sonar3, sonar4, sonar5, sonar6, sonar7, sonar8]

	j=0
	for i in range (norte, 8):
		vetor_sonarN[j] = vetor_sonar[i]
		j+=1
	for a in range(0,norte):
		vetor_sonarN[j] = vetor_sonar[a]
		j+=1
	
	print("")
	#rospy.loginfo("Positioning Vector: %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f \n" % (vetor_sonarN[0], vetor_sonarN[1], vetor_sonarN[2], vetor_sonarN[3], vetor_sonarN[4], vetor_sonarN[5], vetor_sonarN[6], vetor_sonarN[7]))	

	pub_sonarN.publish(vetor_sonarN[0])
	pub_sonarNE.publish(vetor_sonarN[1])
	pub_sonarL.publish(vetor_sonarN[2])
	pub_sonarSE.publish(vetor_sonarN[3])
	pub_sonarS.publish(vetor_sonarN[4])
	pub_sonarSO.publish(vetor_sonarN[5])	
	pub_sonarO.publish(vetor_sonarN[6])
	pub_sonarNO.publish(vetor_sonarN[7])
	pub_AP1toRobot.publish(euclideanDistance1)
	pub_AP2toRobot.publish(euclideanDistance2)
	pub_AP3toRobot.publish(euclideanDistance3)
	pub_AP4toRobot.publish(euclideanDistance4)	
	pub_robotOrientation.publish(yawDegree)

	#rospy.loginfo("Publish -> sonarN sonarNE sonarL sonarSE sonarS sonarSO sonarO sonarNO AP1toRobot AP2toRobot AP3toRobot AP4toRobot robotOrientation")


	r.sleep()


	
	





