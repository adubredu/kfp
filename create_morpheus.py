import pybullet as p
import time
import numpy as np
import math
from datetime import datetime
from datetime import datetime
import pybullet_data
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tools'))

import utils as ut

clid = p.connect(p.GUI)
# p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setGravity(0, 0, -9.81)

floor = p.loadURDF('models/floor/floor.urdf',useFixedBase=True)
morph = p.loadURDF('models/morpheus_description/morpheus.urdf', (0,0,0.9))

for i in range(p.getNumJoints(morph)):
  print(p.getJointInfo(morph, i))

time.sleep(2)
s=0
for t in np.arange(0.0, 0.25, 0.05):
	p.resetJointState(morph, 1, t)
	s+=t/2.
	p.resetJointState(morph, 0, s)
	print(t)
	time.sleep(1)
for t in np.arange(0.0, 0.25, 0.05):
	p.resetJointState(morph, 2, t)
	s+=t/2.
	p.resetJointState(morph, 0, s)
	print(t)
	time.sleep(1)
for t in np.arange(0.0, 0.25, 0.05):
	p.resetJointState(morph, 3, t)
	s+=t/2.
	p.resetJointState(morph, 0, s)
	print(t)
	time.sleep(1)
for t in np.arange(0.0, 0.25, 0.05):
	p.resetJointState(morph, 4, t)
	s+=t/2.
	p.resetJointState(morph, 0, s)
	print(t)
	time.sleep(1)
time.sleep(10)
# for i in range(1000):
# 	p.stepSimulation()
# 	time.sleep(1)