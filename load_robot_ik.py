import pybullet as p
import time
import math
from datetime import datetime
from datetime import datetime
import numpy as np
import pybullet_data

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tools'))

import utils as ut
# import tools.utils as ut

clid = p.connect(p.GUI)
# p.setGravity(0,0,-10)

p.setAdditionalSearchPath('models')
floor = p.loadURDF('floor/floor.urdf',useFixedBase=True)
# ff = p.loadURDF('panda_arm_hand_on_carter_visual.urdf',0,3,0)
ff = p.loadURDF('franka_description/robots/panda_arm_hand.urdf',(0,3,0))
ut.set_point(ff, ut.Point(z=ut.stable_z(ff,floor)))

ffjoints = [12,13,14,15,16,17,18]
print(ff)

# cid = p.createConstraint(husky, -1, panda, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0., 0., -.5],
#                          [0, 0, 0, 1])
# for i in range(p.getNumJoints(ff)):
# 	print(p.getJointInfo(ff, i))

# lower_limits = np.maximum(get_min_limits(robot, free_joints), current_positions - free_deltas)
# upper_limits = np.minimum(get_max_limits(robot, free_joints), current_positions + free_deltas)


# jointPositions = [0.01200158428400755, -0.5697816014289856, 5.6801487517077476e-05,
# 					-2.8105969429016113, -0.00025768374325707555, 3.0363450050354004, 0.7410701513290405]
# for jointIndex in range(7):
# 	p.resetJointState(ob, jointIndex, jointPositions[jointIndex])

eeid = 10
wheels = [5,6,8,10]
wheelVelocities = [0, 0,0,0]
wheelDeltasTurn = [1, -1,1,-1]
wheelDeltasFwd = [1, 1,1,1]
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 0
trailDuration = 15
ll, ul = ut.get_custom_limits(ff, ut.get_movable_joints(ff), {})
t=0.
while 1:
	
	t += 0.001
	for i in range(1):
		# pos = [-0.4,0.2*math.cos(t),0.+0.2*math.sin(t)]
		pos = [0.2 * math.cos(t), 0.8, 0. + 0.2 * math.sin(t) + 0.7]
		#end effector points down, not up (in case useOrientation==1)
		orn = p.getQuaternionFromEuler([0, -math.pi, 0])
		#pos is relative to the robot's base
		jointPoses = p.calculateInverseKinematics(ff,
												  eeid,
												  pos,
												  lowerLimits = ll,
												  upperLimits = ul)
		# jointPoses = ut.sub_inverse_kinematics(ff,0,eeid,(pos,None))
		# print(jointPoses)
		if jointPoses is not None:
			for i in range(7):
				p.resetJointState(ff, i, jointPoses[i])
	ls = p.getLinkState(ff, eeid)
	if (hasPrevPose):
		p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
		p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
	prevPose = pos
	prevPose1 = ls[4]
	hasPrevPose = 1

# for i in range(1000):
# 	time.sleep(1)
# 	p.stepSimulation()