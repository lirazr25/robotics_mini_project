#!/usr/bin/env python
import os
import math
import random
import rospy

world = random.randint(1,6)

Yaw = random.uniform(0,2*math.pi)
xPos = random.uniform(-4,2)
yPos = random.uniform(-6,-3)

os.system("roslaunch project our_komodo.launch world:=" + str(world) + " xPos:=" + str(xPos) + " yPos:=" + str(yPos) + " Yaw:=" + str(Yaw))