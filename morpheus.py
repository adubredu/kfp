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
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath('models')
p.setRealTimeSimulation(1)



class Morpheus:
	def __init__(self):
		kitchen_path = 'kitchen_description/urdf/kitchen_part_right_gen_convex.urdf'
		with ut.HideOutput(enable=True):
			self.floor = p.loadURDF('floor/floor.urdf',useFixedBase=True)
			self.kitchen = p.loadURDF(kitchen_path,[-3,0,0],useFixedBase=True)

		z = ut.stable_z(self.kitchen, self.floor) - ut.get_point(self.floor)[2]
		point = np.array(ut.get_point(self.kitchen)) - np.array([0, 0, z])
		ut.set_point(self.floor, point)
		with ut.HideOutput(enable=True):
			self.husky = p.loadURDF("husky/husky.urdf", [0.290388, 0.329902, -1.110270],
			                   [0.002328, -0.000984, 0.996491, 0.083659])
			self.panda = p.loadURDF("franka_panda/panda.urdf", 0.193749, 0.345564, -0.720208, 0.002327,
		                    -0.000988, 0.996491, 0.083659)
		cid = p.createConstraint(self.husky, 8, self.panda, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0., 0., -.5],
		                         [0, 0, 0, 1])
		self.panda_end_effector = 11
		self.num_joints = 7
		self.wheels = [2, 3, 4, 5]
		self.linear_speed = 3.0
		self.turn_speed = 1.0

		self.move_base_to_position(2,-3,1.57)
		time.sleep(30)


	def calculate_inverse_kinematics(self, targetPos, threshold, maxIter):
		closeEnough = False
		iter = 0
		dist2 = 1e30
		while (not closeEnough and iter < maxIter):
			jointPoses = p.calculateInverseKinematics(self.panda, self.panda_end_effector, targetPos)
			for i in range(self.num_joints):
				p.resetJointState(self.panda, i, jointPoses[i])
			ls = p.getLinkState(self.panda, self.panda_end_effector)
			newPos = ls[4]
			diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
			dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
			closeEnough = (dist2 < threshold)
			iter = iter + 1

		return jointPoses


	def orient_base_to_yaw(self, theta, tolerance=0.1):
		pose, orientation = p.getBasePositionAndOrientation(self.husky)
		yaw = p.getEulerFromQuaternion(orientation)[2]
		wheelVelocities = [0, 0, 0, 0]
		wheelDeltasTurn = [1, -1, 1, -1]
		wheelDeltasFwd = [1, 1, 1, 1]

		while np.abs(yaw - theta) > tolerance:
			wheelVelocities = [0, 0, 0, 0]
			if yaw > theta:
				#turn clockwise
				for i in range(len(self.wheels)):
					wheelVelocities[i] = wheelVelocities[i] + self.turn_speed * wheelDeltasTurn[i]

			else:
				#turn anti-clockwise
				for i in range(len(self.wheels)):
					wheelVelocities[i] = wheelVelocities[i] - self.turn_speed * wheelDeltasTurn[i]

			for i in range(len(self.wheels)):
			    p.setJointMotorControl2(self.husky,
			                            self.wheels[i],
			                            p.VELOCITY_CONTROL,
			                            targetVelocity=wheelVelocities[i],
			                            force=1000)

			pose, orientation = p.getBasePositionAndOrientation(self.husky)
			yaw = p.getEulerFromQuaternion(orientation)[2]
			print(yaw)


	def drive_base_for_distance(self, goal_distance, tolerance=0.1):
		init_pos, orientation = p.getBasePositionAndOrientation(self.husky)
		
		wheelVelocities = [0, 0, 0, 0]
		wheelDeltasTurn = [1, -1, 1, -1]
		wheelDeltasFwd = [1, 1, 1, 1]

		dist_covered = 0

		while np.abs(dist_covered - goal_distance) > tolerance:
			wheelVelocities = [0, 0, 0, 0]
			for i in range(len(self.wheels)):
				wheelVelocities[i] = wheelVelocities[i] + self.linear_speed * wheelDeltasFwd[i]

			
			for i in range(len(self.wheels)):
			    p.setJointMotorControl2(self.husky,
			                            self.wheels[i],
			                            p.VELOCITY_CONTROL,
			                            targetVelocity=wheelVelocities[i],
			                            force=1000)

			pose, orientation = p.getBasePositionAndOrientation(self.husky)
			dist_covered = np.sqrt((init_pos[0]-pose[0])**2 + (init_pos[1]-pose[1])**2)
			print(dist_covered)


	def move_base_to_position(self, gx, gy, theta):
		init_pos, orientation = p.getBasePositionAndOrientation(self.husky)
		direction = np.arctan2((gy-init_pos[1]), (gx-init_pos[0]))
		self.orient_base_to_yaw(direction)
		distance = np.sqrt((gx-init_pos[0])**2 + (gy-init_pos[1])**2)
		self.drive_base_for_distance(distance)
		self.orient_base_to_yaw(theta)



	def run_draw_circle_test(self):
		t = 0.
		prevPose = [0, 0, 0]
		prevPose1 = [0, 0, 0]
		hasPrevPose = 0
		useNullSpace = 0

		useOrientation = 0
		useSimulation = 0
		useRealTimeSimulation = 1
		p.setRealTimeSimulation(useRealTimeSimulation)
		trailDuration = 15
		basepos = [0, 0, 0]
		ang = 0
		ang = 0

		wheels = [2, 3, 4, 5]
		wheelVelocities = [0, 0, 0, 0]
		wheelDeltasTurn = [1, -1, 1, -1]
		wheelDeltasFwd = [1, 1, 1, 1]
		time.sleep(5)

		while 1:
			keys = p.getKeyboardEvents()
			shift = 0.01
			wheelVelocities = [0, 0, 0, 0]
			self.speed = 1.0
			for k in keys:
				if p.B3G_LEFT_ARROW in keys:
					for i in range(len(wheels)):
						wheelVelocities[i] = wheelVelocities[i] - self.speed * wheelDeltasTurn[i]
				if p.B3G_RIGHT_ARROW in keys:
					for i in range(len(wheels)):
						wheelVelocities[i] = wheelVelocities[i] + self.speed * wheelDeltasTurn[i]
				if p.B3G_UP_ARROW in keys:
					for i in range(len(wheels)):
						wheelVelocities[i] = wheelVelocities[i] + self.speed * wheelDeltasFwd[i]
				if p.B3G_DOWN_ARROW in keys:
					for i in range(len(wheels)):
						wheelVelocities[i] = wheelVelocities[i] - self.speed * wheelDeltasFwd[i]

			baseorn = p.getQuaternionFromEuler([0, 0, ang])
			for i in range(len(wheels)):
				p.setJointMotorControl2(self.husky,
			                        wheels[i],
			                        p.VELOCITY_CONTROL,
			                        targetVelocity=wheelVelocities[i],
			                        force=1000)

			if (useRealTimeSimulation):
				t = time.time()
			else:
				t = t + 0.001

			pos = [0.2 * math.cos(t), 0, 0. + 0.2 * math.sin(t) + 0.]
			orn = p.getQuaternionFromEuler([0, -math.pi, 0])
			threshold = 0.001
			maxIter = 100
			jointPoses = self.calculate_inverse_kinematics(pos, threshold, maxIter)

			for i in range(self.num_joints):
				p.resetJointState(self.panda, i, jointPoses[i])
			ls = p.getLinkState(self.panda, self.panda_end_effector)
			if (hasPrevPose):
				p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
				p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
			prevPose = pos
			prevPose1 = ls[4]
			hasPrevPose = 1
			self.orient_base_to_yaw(1)


	def run_move_square_test(self):
		self.orient_base_to_yaw(1.57)
		time.sleep(2)
		self.drive_base_for_distance(1)
		time.sleep(2)
		self.orient_base_to_yaw(0)
		time.sleep(2)
		self.drive_base_for_distance(1)
		time.sleep(2)
		self.orient_base_to_yaw(-1.57)
		time.sleep(2)
		self.drive_base_for_distance(1)
		time.sleep(2)
		self.orient_base_to_yaw(3.14)
		time.sleep(2)
		self.drive_base_for_distance(1)
		time.sleep(2)
		self.orient_base_to_yaw(1.57)
		time.sleep(2)
		time.sleep(15)


if __name__ == '__main__':
	m = Morpheus()