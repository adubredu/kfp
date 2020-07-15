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
		self.panda_fingers_index = [9,10]
		self.panda_fingers_limits= [0.0, 0.04]
		self.num_joints = 7
		self.wheels = [2, 3, 4, 5]
		self.linear_speed = 3.0
		self.turn_speed = 0.5

		self.cabinet_level_arm_joints = [0.11303627351105561, 1.7194943220174232, 0.004610986775859532, -0.35555030977842883, -1.9527160082703017, 2.995056478815023, 1.6548123762061726]
		self.cabinet_base_pose = ((-1.870035902802267, 1.4581147943665136, -1.4776955716029512), (-0.00019330860728466418, 0.00025927680860549996, 0.998695377102498, -0.051063090010477596))
		self.top_drawer_handle_close_pose =  [[-2.700021551986831, 1.2222981416726535, -0.61743105716402], [-0.48237835221694003, 0.6950004414191295, 0.2741104699837661, -0.4573280682234183]]
		self.door_indices = {
						'chewie_door_right':18,
						'chewie_door_left':22,
						'dagger_door_left':27,
						'dagger_door_right':31,
						'hitman_drawer_top':37,
						'hitman_drawer_bottom':40,
						'indigo_door_right':48,
						'indigo_door_left':53,
						'indigo_drawer_top':56,
						'indigo_drawer_bottom':58,
						'baker':14
		}
		self.run_open_drawer_test()
		# self.arm_teleop()
		# for i in range(p.getNumJoints(self.panda)):
		# 	print(p.getJointInfo(self.panda,i))
		# self.move_arm_to_cabinet_level()
		# self.run_gripper_test()
		# self.set_gripper_to(0.02)
		# self.at_cabinet_pose = ((-2.1500243687136127, 1.092980084593978, -1.4781747553214735), 
		# 	(5.966205995056604e-05, 0.0009687922557074192, 0.7333478024202129, 0.6798529683449563))
		# self.move_base_to_position(2,-3,1.57)
		# self.run_draw_circle_test()
		# self.move_base_to_position(self.at_cabinet_pose[0][0],self.at_cabinet_pose[0][1],
		# 					1.57)
		time.sleep(30)



	def calculate_inverse_kinematics(self, targetPos,targetOr, threshold, maxIter):
		closeEnough = False
		iter = 0
		dist2 = 1e30
		while (not closeEnough and iter < maxIter):
			jointPoses = p.calculateInverseKinematics(self.panda, self.panda_end_effector, targetPos,targetOr)
			for i in range(self.num_joints):
				p.resetJointState(self.panda, i, jointPoses[i])
			ls = p.getLinkState(self.panda, self.panda_end_effector)
			newPos = ls[4]
			diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
			dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
			closeEnough = (dist2 < threshold)
			iter = iter + 1

		return jointPoses


	def move_arm_to_cabinet_level(self):
		joints = [0,1,2,3,4,5,6]
		p.setJointMotorControlArray(self.panda, joints, controlMode=p.POSITION_CONTROL,
									targetPositions=self.cabinet_arm_level)
		p.stepSimulation()


	def close_gripper(self):
		p.setJointMotorControl2(self.panda, self.panda_fingers_index[0],controlMode=p.POSITION_CONTROL,targetPosition=self.panda_fingers_limits[0])
		p.setJointMotorControl2(self.panda, self.panda_fingers_index[1],controlMode=p.POSITION_CONTROL,targetPosition=self.panda_fingers_limits[0])
		p.stepSimulation()


	def open_gripper(self):
		p.setJointMotorControl2(self.panda, self.panda_fingers_index[0],controlMode=p.POSITION_CONTROL,targetPosition=self.panda_fingers_limits[1])
		p.setJointMotorControl2(self.panda, self.panda_fingers_index[1],controlMode=p.POSITION_CONTROL,targetPosition=self.panda_fingers_limits[1])
		p.stepSimulation()

	def set_gripper_to(self,value):
		p.setJointMotorControl2(self.panda, self.panda_fingers_index[0],controlMode=p.POSITION_CONTROL,targetPosition=value)
		p.setJointMotorControl2(self.panda, self.panda_fingers_index[1],controlMode=p.POSITION_CONTROL,targetPosition=value)
		p.stepSimulation()

	def orient_base_to_yaw(self, theta, tolerance=0.1, intelligent=True):
		pose, orientation = p.getBasePositionAndOrientation(self.husky)
		yaw = p.getEulerFromQuaternion(orientation)[2]
		wheelVelocities = [0, 0, 0, 0]
		wheelDeltasTurn = [1, -1, 1, -1]
		wheelDeltasFwd = [1, 1, 1, 1]

		if not intelligent:

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
				time.sleep(1)
				pose, orientation = p.getBasePositionAndOrientation(self.husky)
				yaw = p.getEulerFromQuaternion(orientation)[2]
				print('turning: ',yaw)
		else:
			if np.abs(theta) > 1.57 and yaw > 1.57:
				while np.abs(yaw - theta) > tolerance:
					wheelVelocities = [0, 0, 0, 0]
					if yaw < theta:
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
					time.sleep(1)
					pose, orientation = p.getBasePositionAndOrientation(self.husky)
					yaw = p.getEulerFromQuaternion(orientation)[2]
					print('turning: ',yaw)
		print('done yawing')
		for i in range(len(self.wheels)):
			    p.setJointMotorControl2(self.husky,
			                            self.wheels[i],
			                            p.VELOCITY_CONTROL,
			                            targetVelocity=0.0,
			                            force=1000)
		


	def drive_base_for_distance(self, goal_distance, tolerance=0.1):
		init_pos, orientation = p.getBasePositionAndOrientation(self.husky)
		
		wheelVelocities = [0, 0, 0, 0]
		wheelDeltasTurn = [1, -1, 1, -1]
		wheelDeltasFwd = [1, 1, 1, 1]

		dist_covered = 0

		while np.abs(dist_covered - np.abs(goal_distance)) > tolerance:
			wheelVelocities = [0, 0, 0, 0]
			if dist_covered < goal_distance:
				for i in range(len(self.wheels)):
					wheelVelocities[i] = wheelVelocities[i] + self.linear_speed * wheelDeltasFwd[i]
			else:
				for i in range(len(self.wheels)):
					wheelVelocities[i] = wheelVelocities[i] - self.linear_speed * wheelDeltasFwd[i]

			
			for i in range(len(self.wheels)):
			    p.setJointMotorControl2(self.husky,
			                            self.wheels[i],
			                            p.VELOCITY_CONTROL,
			                            targetVelocity=wheelVelocities[i],
			                            force=1000)

			pose, orientation = p.getBasePositionAndOrientation(self.husky)
			dist_covered = np.sqrt((init_pos[0]-pose[0])**2 + (init_pos[1]-pose[1])**2)
			print('linear: ',dist_covered)
		print('done translating')
		for i in range(len(self.wheels)):
			    p.setJointMotorControl2(self.husky,
			                            self.wheels[i],
			                            p.VELOCITY_CONTROL,
			                            targetVelocity=0.0,
			                            force=1000)


	def move_base_to_position(self, gx, gy, theta):
		init_pos, orientation = p.getBasePositionAndOrientation(self.husky)
		direction = np.arctan2((gy-init_pos[1]), (gx-init_pos[0]))
		self.orient_base_to_yaw(direction)
		time.sleep(1)
		distance = np.sqrt((gx-init_pos[0])**2 + (gy-init_pos[1])**2)
		self.drive_base_for_distance(distance)
		time.sleep(1)
		# self.orient_base_to_yaw(theta)

		# init_pos, orientation = p.getBasePositionAndOrientation(self.husky)
		# distance = np.sqrt((gx-init_pos[0])**2 + (gy-init_pos[1])**2)
		# self.drive_base_for_distance(distance)



	def open_conf(self, joint):
		joint_name = ut.get_joint_name(self.kitchen, joint)
		if 'left' in joint_name:
			open_position = ut.get_min_limit(self.kitchen, joint)
		else:
			open_position = ut.get_max_limit(self.kitchen, joint)
		if joint_name in ut.CABINET_JOINTS:
			return ut.CABINET_OPEN_ANGLE * open_position / abs(open_position)
		if joint_name in ut.DRAWER_JOINTS:
			return ut.DRAWER_OPEN_FRACTION * open_position
		return open_position


	def closed_conf(self, joint):
		lower, upper = ut.get_joint_limits(self.kitchen, joint)
		if 'drawer' in ut.get_joint_name(self.kitchen, joint):
			fraction = 0.9
			return fraction*lower + (1-fraction)*upper
		if 'left' in ut.get_joint_name(self.kitchen, joint):
			return upper
		return lower


	def open_door(self, name):
		index = self.door_indices[name]
		ut.set_joint_position(self.kitchen, index, self.open_conf(index))


	def close_door(self, name):
		index = self.door_indices[name]
		ut.set_joint_position(self.kitchen, index, self.closed_conf(index))


	def control_joint(self, joint, value, minn, maxx):
		if value < minn:
			value = minn
		if value > maxx:
			value = maxx
		p.setJointMotorControl2(self.panda, joint,
					controlMode=p.POSITION_CONTROL,targetPosition=value)
		p.stepSimulation()


	def arm_teleop(self):
		joint_limits={
						0: (-2.9671, 2.9671),
						1: (-1.8326, 1.8326),
						2: (-2.9671, 2.9671),
						3: (-3.1416, 0.0),
						4: (-2.9671, 2.9671),
						5: (-0.0873, 3.8223),
						6: (-2.9671, 2.9671),
						7: (0.0, -1.0,)
					}
		wheels = [2, 3, 4, 5]
		wheelVelocities = [0, 0, 0, 0]
		wheelDeltasTurn = [1, -1, 1, -1]
		wheelDeltasFwd = [1, 1, 1, 1]

		while 1:
			wheelVelocities = [0, 0, 0, 0]
			keys = p.getKeyboardEvents()
			if ord('q') in keys:
				ji = 0
				currjoint = p.getJointState(self.panda, ji)[0]
				self.control_joint(ji, currjoint-0.01, 
								joint_limits[ji][0],joint_limits[ji][1])
			if ord('w') in keys:
				ji = 0
				currjoint = p.getJointState(self.panda, ji)[0]
				self.control_joint(ji, currjoint+0.01, 
								joint_limits[ji][0],joint_limits[ji][1])
			if ord('a') in keys:
				ji = 1
				currjoint = p.getJointState(self.panda, ji)[0]
				self.control_joint(ji, currjoint-0.01, 
								joint_limits[ji][0],joint_limits[ji][1])
			if ord('s') in keys:
				ji = 1
				currjoint = p.getJointState(self.panda, ji)[0]
				self.control_joint(ji, currjoint+0.01, 
								joint_limits[ji][0],joint_limits[ji][1])
			if ord('z') in keys:
				ji = 2
				currjoint = p.getJointState(self.panda, ji)[0]
				self.control_joint(ji, currjoint-0.01, 
								joint_limits[ji][0],joint_limits[ji][1])
			if ord('x') in keys:
				ji = 2
				currjoint = p.getJointState(self.panda, ji)[0]
				self.control_joint(ji, currjoint+0.01, 
								joint_limits[ji][0],joint_limits[ji][1])
			if ord('e') in keys:
				ji = 3
				currjoint = p.getJointState(self.panda, ji)[0]
				self.control_joint(ji, currjoint-0.01, 
								joint_limits[ji][0],joint_limits[ji][1])
			if ord('r') in keys:
				ji = 3
				currjoint = p.getJointState(self.panda, ji)[0]
				self.control_joint(ji, currjoint+0.01, 
								joint_limits[ji][0],joint_limits[ji][1])
			if ord('d') in keys:
				ji = 4
				currjoint = p.getJointState(self.panda, ji)[0]
				self.control_joint(ji, currjoint-0.01, 
								joint_limits[ji][0],joint_limits[ji][1])
			if ord('f') in keys:
				ji = 4
				currjoint = p.getJointState(self.panda, ji)[0]
				self.control_joint(ji, currjoint+0.01, 
								joint_limits[ji][0],joint_limits[ji][1])
			if ord('c') in keys:
				ji = 5
				currjoint = p.getJointState(self.panda, ji)[0]
				self.control_joint(ji, currjoint-0.01, 
								joint_limits[ji][0],joint_limits[ji][1])
			if ord('v') in keys:
				ji = 5
				currjoint = p.getJointState(self.panda, ji)[0]
				self.control_joint(ji, currjoint+0.01, 
								joint_limits[ji][0],joint_limits[ji][1])
			if ord('t') in keys:
				ji = 6
				currjoint = p.getJointState(self.panda, ji)[0]
				self.control_joint(ji, currjoint-0.01, 
								joint_limits[ji][0],joint_limits[ji][1])
			if ord('y') in keys:
				ji = 6
				currjoint = p.getJointState(self.panda, ji)[0]
				self.control_joint(ji, currjoint+0.01, 
								joint_limits[ji][0],joint_limits[ji][1])
			if ord('g') in keys:
				ji = 7
				currjoint = p.getJointState(self.panda, ji)[0]
				self.control_joint(ji, currjoint-0.01, 
								joint_limits[ji][0],joint_limits[ji][1])
			if ord('h') in keys:
				ji = 7
				currjoint = p.getJointState(self.panda, ji)[0]
				self.control_joint(ji, currjoint+0.01, 
								joint_limits[ji][0],joint_limits[ji][1])
			if p.B3G_LEFT_ARROW in keys:
				for i in range(len(wheels)):
					wheelVelocities[i] = wheelVelocities[i] - self.linear_speed * wheelDeltasTurn[i]
			if p.B3G_RIGHT_ARROW in keys:
				for i in range(len(wheels)):
					wheelVelocities[i] = wheelVelocities[i] + self.linear_speed * wheelDeltasTurn[i]
			if p.B3G_UP_ARROW in keys:
				for i in range(len(wheels)):
					wheelVelocities[i] = wheelVelocities[i] + self.linear_speed * wheelDeltasFwd[i]
			if p.B3G_DOWN_ARROW in keys:
				for i in range(len(wheels)):
					wheelVelocities[i] = wheelVelocities[i] - self.linear_speed * wheelDeltasFwd[i]

			for i in range(len(wheels)):
				p.setJointMotorControl2(self.husky,
			                        wheels[i],
			                        p.VELOCITY_CONTROL,
			                        targetVelocity=wheelVelocities[i],
			                        force=1000)
			# p.stepSimulation()

			joint_angles=[]
			for i in joint_limits:
				joint_angles.append(p.getJointState(self.panda,i)[0])
			print('JOINT ANGLES ARE: ',joint_angles)
			pose = p.getBasePositionAndOrientation(self.husky)
			print('BASE POSE AND ORIENTATION: ',pose)
			eepose = p.getLinkState(self.panda, self.panda_end_effector)
			print('WORLD END-EFFECTOR POSE: ',eepose[0],eepose[1])
			print('&'*30)
			print(' ')


	def run_open_drawer_test(self):
		self.open_gripper()
		theta = p.getEulerFromQuaternion(self.cabinet_base_pose[1])[2]
		self.move_base_to_position(self.cabinet_base_pose[0][0]+0.1,
			self.cabinet_base_pose[0][1],theta)
		time.sleep(1)
		joints = [0,1,2,3,4,5,6]
		angles= self.calculate_inverse_kinematics(self.top_drawer_handle_close_pose[0],
					self.top_drawer_handle_close_pose[1], threshold=0.001, maxIter=100)
		p.setJointMotorControlArray(self.panda, joints, controlMode=p.POSITION_CONTROL,
									targetPositions=angles[:7])
		time.sleep(1)
		self.close_gripper()
		time.sleep(2)
		self.drive_base_for_distance(-0.5)
		p.stepSimulation()





	def run_open_doors_test(self):
		for key in self.door_indices:
			self.open_door(key)
		time.sleep(5)

		for key in self.door_indices:
			self.close_door(key)
		time.sleep(30)


	def run_gripper_test(self):
		self.open_gripper()
		time.sleep(5)
		self.close_gripper()
		time.sleep(5)
		self.open_gripper()
		time.sleep(5)
		self.close_gripper()
		time.sleep(5)
		self.open_gripper()
		time.sleep(5)
		self.close_gripper()
		time.sleep(5)
		self.open_gripper()
		time.sleep(5)
		self.close_gripper()
		time.sleep(5)
		self.open_gripper()
		time.sleep(5)
		self.close_gripper()

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
			# jointPoses = self.calculate_inverse_kinematics(pos, threshold, maxIter)

			# for i in range(self.num_joints):
			# 	p.resetJointState(self.panda, i, jointPoses[i])
			# ls = p.getLinkState(self.panda, self.panda_end_effector)
			# if (hasPrevPose):
			# 	p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
			# 	p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
			# prevPose = pos
			# prevPose1 = ls[4]
			# hasPrevPose = 1
			print(p.getBasePositionAndOrientation(self.husky))
			# self.orient_base_to_yaw(1)


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