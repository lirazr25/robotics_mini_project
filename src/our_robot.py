#!/usr/bin/env python

import rospy, cv2, numpy, math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64

zone_printing = 1
printing = 1 # 1 = print, 0 = don't
cube_order = [[0,1,2],[0,2,1],[1,0,2],[1,2,0],[2,0,1],[2,1,0]]
our_order = None

RED_CUBE = 0
BLUE_CUBE = 1
GREEN_CUBE = 2

FOUND = None # when scanning begins will store the cubes seen
finished = False
last_cube = False

img = None
current_cube = 0 # is an index!! the current color is in our_order[current_cube]
NOISE_RANGE = 25

should_detect = True
pub = rospy.Publisher('/komodo_1/diff_driver/command', Twist, queue_size = 10)

ranges = None # will represent laser, will get updated with every laser scan
safety = 0.5
min_dist = 0
closest = 1

first_spin = 0
rotated = 0
last_direction = ""

c_zone = 0
c_nzone = 0
smove = False
first_is_far = False
got_to_far_zone = False
has_obstacle = False

# for our victory dance
shoulder = rospy.Publisher('/komodo_1/shoulder_controller/command', Float64, queue_size=10)
elbow = rospy.Publisher('/komodo_1/elbow1_controller/command', Float64, queue_size=10)
base = rospy.Publisher('/komodo_1/base_rotation_controller/command', Float64, queue_size=10) 


def init():
	rospy.init_node('our_robot', anonymous = True)
	rospy.Subscriber('/komodo_1/Front_Camera/image_raw', Image, detector)
	rospy.Subscriber("/komodo_1/scan", LaserScan, scanner)

	# subscribing to string messages by ball detector. when recieved activates detector(= detectObjects)

def scanner(laser):
	global ranges, closest, min_dist
	ranges = laser.ranges
	min_dist = min(ranges[len(ranges)/2-180:len(ranges)/2+180])
	closest = min(closest,min(ranges))
	# min_dist is the distance to the nearest object in front of us

def detector(Image):
	global zone_printing, printing, has_obstacle, last_direction, got_to_far_zone, first_is_far, pub, should_detect, current_cube, finished, rotated, first_spin,FOUND,smove, last_cube
	booe = False
	img = Image
	rbg = [([0,70,50],[10,255,255]),([110,50,50],[130,255,255]),([45,100,50],[75,255,255])]
	#first is red, second is blue, third is green
	# the rgb boundaries are in hsv! do not confuse!
	bridge = CvBridge()
	#bridge used to translate cv image to ros image
	cv_image = bridge.imgmsg_to_cv2(img,"bgr8")
	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
	# easier to work with with online manuals and numpy
	if should_detect:
		# we would like to know where each cube is
		red_mask = numpy.asarray(cv2.inRange(hsv, numpy.array(rbg[0][0], dtype = "uint8"), numpy.array(rbg[0][1], dtype = "uint8")))
		blue_mask = numpy.asarray(cv2.inRange(hsv, numpy.array(rbg[1][0], dtype = "uint8"), numpy.array(rbg[1][1], dtype = "uint8")))
		green_mask = numpy.asarray(cv2.inRange(hsv, numpy.array(rbg[2][0], dtype = "uint8"), numpy.array(rbg[2][1], dtype = "uint8")))
		cv2.imwrite('/users/studs/bsc/2011/rihnshty/catkin_ws/src/project/scripts/rgb.jpg', cv_image)
		cv2.imwrite('/users/studs/bsc/2011/rihnshty/catkin_ws/src/project/scripts/hsv.jpg', hsv)
		redCoordList = numpy.argwhere(red_mask==255)
		blueCoordList = numpy.argwhere(blue_mask==255)
		greenCoordList = numpy.argwhere(green_mask==255)
		if cv2.countNonZero(red_mask) > 1000 and cv2.countNonZero(blue_mask) > 1000:
			# saw red and blue, needs to figure which is where
			print '\nSaw RED & BLUE!'
			if redCoordList[0][1] > blueCoordList[0][1]:
				FOUND = [BLUE_CUBE,RED_CUBE,GREEN_CUBE]
			else:
				FOUND = [RED_CUBE,BLUE_CUBE,GREEN_CUBE]
			should_detect = False
			print 'Printing cubes by their zones:\n',FOUND
			printing = 1
			rotated = 0
			move("STRAIGHT",0)
			rospy.sleep(1)
		elif cv2.countNonZero(red_mask) > 1000 and cv2.countNonZero(green_mask) > 1000:
			# saw red and green
			print '\nSaw RED & GREEN!'
			if redCoordList[0][1] > greenCoordList[0][1]:
				FOUND = [GREEN_CUBE,RED_CUBE,BLUE_CUBE]
			else:
				FOUND = [RED_CUBE,GREEN_CUBE,BLUE_CUBE]
			should_detect = False
			print 'Printing cubes by their zones:\n', FOUND
			printing = 1
			rotated = 0
			move("STRAIGHT",0)
			rospy.sleep(1)
		elif cv2.countNonZero(blue_mask) > 1000 and cv2.countNonZero(green_mask) > 1000:
			# saw blue and green
			print '\nSaw BLUE & GREEN!'
			if blueCoordList[0][1] > greenCoordList[0][1]:
				FOUND = [GREEN_CUBE,BLUE_CUBE,RED_CUBE]
			else:
				FOUND = [BLUE_CUBE,GREEN_CUBE,RED_CUBE]
			should_detect = False
			print 'Printing cubes by their zones:\n', FOUND
			printing = 1
			move("STRAIGHT",0)
			rotated = 0
			rospy.sleep(1)
		else:
			if printing == 1:
				print 'Saw NOTHING!'
				printing = 0
			move("LEFT",0.4)
			rotated+=1
			if rotated>=250:
				rotated = 0
				first_spin += 1
				if (first_spin % 2) == 1:
					print 'Finished circling. Moving forward'
					move("STRAIGHT",0.3)
					rospy.sleep(first_spin*closest)
				else:
					print 'Finished circling. Moving backward'
					move("STRAIGHT",-0.3)
					rospy.sleep(first_spin*closest)
	elif current_cube == 0:
		# we would like to know where we headed
		mask = numpy.asarray(cv2.inRange(hsv, numpy.array(rbg[our_order[current_cube]][0], dtype = "uint8"), numpy.array(rbg[our_order[current_cube]][1], dtype = "uint8")))
		coordList = numpy.argwhere(mask == 255)
		if cv2.countNonZero(mask) > 1000:
			direction = findDirection(mask, coordList)
			if printing == 1:
				print '\nSaw current color!'
				print 'Moving',direction
				printing = 0
			if direction != "STRAIGHT":
				last_direction = direction
			# should move in direction
			if has_obstacle:
				direction = "STRAIGHT"
				distance = 0.3
				has_obstacle = False
			elif min_dist < safety and direction == "STRAIGHT":
				if max(ranges[len(ranges)/2-120:len(ranges)/2+120]) >= 2.5:
					print 'Obstacle ahead! Turning',last_direction.lower()
					printing = 1	
					direction = last_direction
					distance = 0.3
					has_obstacle = True
				else:
					print 'Got to first cube!'
					printing = 1
					current_cube += 1
					distance = -0.4
					smove = False
					rotated = 0
					move(direction,distance)
					rospy.sleep(2)
			else:
				distance = 0.3
			move(direction,distance)
		elif FOUND.index(our_order[current_cube]) == 2: # first cube is in the far zone
			first_is_far = True
			if got_to_far_zone:
				if printing == 1:
					print 'Got to the far zone! Searching for cube...'
					printing = 0
				move("RIGHT",0.4)
			else:
				if printing == 1:
					print '\nGoing to the left zone!'
					printing = 0
				left_zone_mask = numpy.asarray(cv2.inRange(hsv, numpy.array(rbg[FOUND[0]][0], dtype = "uint8"), numpy.array(rbg[FOUND[0]][1], dtype = "uint8")))
				left_zone_coordList = numpy.argwhere(left_zone_mask == 255)
				direction = findDirection(left_zone_mask, left_zone_coordList)
				#print direction
				# should move in direction
				if min_dist < 1 and direction == "STRAIGHT":
					print 'Got to left zone!\nGoing to the far zone!'
					printing = 1
					got_to_far_zone = True
					distance = 0
					gotonextzone()
				else:
					distance = 0.3
				move(direction,distance)
		else: # means we need to get to the far zone
			if printing == 1:
				print '\nSaw no color!'
				printing = 0
			# should start rotating
			if min_dist < safety:
				move("RIGHT",0.4)
			else:
				move("STRAIGHT",0.3)
	elif current_cube < 3:
		if current_cube == 2 and not last_cube:
			printing = 1
			zone_printing = 1
			move ("STRAIGHT",0)
			last_cube = True
			smove = False
			rospy.sleep(2)
		gotonextzone()
		if zone_printing == 1:
			print 'current zone is:',c_zone,'next zone is:', c_nzone
			zone_printing = 0
		mask = numpy.asarray(cv2.inRange(hsv, numpy.array(rbg[our_order[current_cube]][0], dtype = "uint8"), numpy.array(rbg[our_order[current_cube]][1], dtype = "uint8")))
		coordList = numpy.argwhere(mask == 255)
		if cv2.countNonZero(mask) > 1000:
			direction = findDirection(mask, coordList)
			if printing == 1:
				print '\nSaw current color!'
				print 'Moving',direction.lower()
				printing = 0
			if direction != "STRAIGHT":
				last_direction = direction
			# should move in direction
			if has_obstacle:
				direction = "STRAIGHT"
				distance = 0.3
				has_obstacle = False
			elif min_dist < safety and direction == "STRAIGHT":
				if max(ranges[len(ranges)/2-120:len(ranges)/2+120]) >= 2.5:
					if printing == 1:
						print 'Obstacle ahead! Moving',last_direction.lower()
						printing = 0
					direction = last_direction
					distance = 0.3
					has_obstacle = True
				else:
					if last_cube:
						print 'Got to the last cube!'
					else:
						print 'Got to the second cube!'
					printing = 1
					current_cube += 1
					distance = -0.4
					if last_cube:
						distance = 0
					move(direction,distance)
					rotated = 0
					rospy.sleep(2)	
			else:
				distance = 0.3
			move(direction,distance)
		else: # means we need to get to the far zone
			#print '\nSaw no color!'
			# should start rotating
			move("RIGHT",0.4)
	else:
		move("RIGHT",0.5)
		rotated += 1
		dance = Float64()
		dance.data = 1
		elbow.publish(dance)
		rospy.sleep(1)
		shoulder.publish(dance)
		rospy.sleep(1)
		dance.data = -1
		elbow.publish(dance)
		rospy.sleep(1)
		shoulder.publish(dance)
		rospy.sleep(1)
		#shoulder.publish(dance)
		#elbow.publish(dance)
		#swingarm()
		if rotated == 10:
			finished = True
			move("STRAIGHT",0)
			print 'Saionara Mr. robot!'
			rospy.sleep(2)

def swingarm():
	swing = Float64()
	for i in range (1,5):
		swing.data = 4
		base.publish(swing)
		rospy.sleep(0.5)
		swing.data = -4
		base.publish(swing)
		rospy.sleep(0.5)

def gotonextzone():
	global smove,c_zone,c_nzone, first_is_far
	c_zone = FOUND.index(our_order[current_cube-1])
	c_nzone = FOUND.index(our_order[current_cube])
	#angle = 3
	direction = "CIRCLE"

	if c_zone == 0 and c_nzone == 2:
		angle = 15
	elif c_zone == 0 and c_nzone == 1:
		angle = 3
	elif c_zone == 1 and c_nzone == 2:
		angle = 18
	elif c_zone == 1 and c_nzone == 0:
		angle = 3
		direction = "BCIRCLE"
	elif c_zone == 2 and c_nzone == 0:
		angle = 16
		direction = "BCIRCLE"
	elif c_zone == 2 and c_nzone == 1:
		angle = 14
		direction = "BCIRCLE"
	if first_is_far:
		angle = 14
		first_is_far = False
	if smove == False:
		move(direction,0.5)
		smove = True
		rospy.sleep(angle)

def move(direction,distance):
	twist = Twist()
	twist.linear.x = 0 # moving direction
	twist.linear.y = 0 # no
	twist.linear.z = 0 # no
	twist.angular.x = 0 # no
	twist.angular.y = 0 # no
	twist.angular.z = 0 # rotation angle

	if direction == "LEFT":
		twist.angular.z = distance
	elif direction == "RIGHT":
		twist.angular.z = -distance
	elif direction == "CIRCLE":
		twist.angular.z = distance/1.2
		twist.linear.x = -distance*2
	elif direction == "BCIRCLE":
		twist.angular.z = -distance/1.2
		twist.linear.x = -distance*2
	else:
		twist.linear.x = distance
	pub.publish(twist)
	#print 'moving',direction,":",distance


def findDirection(mask,coordList):
	global NOISE_RANGE
	left = mask.shape[1] # width of image (shape[0] is length)
	for i in range(0,coordList.shape[0]-NOISE_RANGE):
		col = coordList[i][1]
		if col < left:
			row = coordList[i][0]
			noise = False
			for j in range(i + 1, i + NOISE_RANGE + 1):
				if (not(row == coordList[j][0] and col + j - i == coordList[j][1])):
					noise = True
			if not(noise):
				left = col
	right = -1
	for i in range(coordList.shape[0] - 1,NOISE_RANGE - 1, -1):
		col = coordList[i][1]
		if col > right:
			row = coordList[i][0]
			noise = False
			for j in range(i - 1,i - NOISE_RANGE - 1,-1):
				if (not(row == coordList[j][0] and col + j - i == coordList[j][1])):
					noise = True
			if not(noise):
				right = col
	mid = mask.shape[1]/2
	#print '\nmid:', mid, '\nleft:', left, '\nright:', right
	if left != mask.shape[1] and right != -1 and right < mask.shape[1]:
		if (left <= mid and right >= mid):
			return "STRAIGHT"
		elif (left>mid):
			return "RIGHT"
		elif (right<mid):
			return "LEFT"
	return last_direction


def main():
	global our_order
	rospy.sleep(12)
	user_in = raw_input('\nPlease select order to go by:\n1. Red->Blue->Green\n2. Red->Green->Blue\n3. Blue->Red->Green\n4. Blue->Green->Red\n5. Green->Red->Blue\n6. Green->Blue->Red\n')
	our_order = cube_order[int(user_in)-1]
	try:
		init()
		rate = rospy.Rate(1)
		while not rospy.is_shutdown() and not(finished):
			rate.sleep()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main()