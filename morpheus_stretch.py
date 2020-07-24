import pybullet as p
import time
import numpy as np
import math
from datetime import datetime
from datetime import datetime
import pybullet_data
import sys
import os
from grocery_items import Grocery_item
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
		# time.sleep(5)
		with ut.HideOutput(enable=True):
			self.morph = p.loadURDF("morpheus_description/morph_with_horizontal_gripper.urdf", [-.5,6,-1.45])

		self.setup_environment()

		self.morph_end_effector = 11
		self.morph_fingers_index = [16,17]
		# self.morph_fingers_limits= [0.0, 0.04]
		self.num_joints = 7
		self.wheels = [2, 3, 4, 5]
		self.linear_speed = 3.0
		self.turn_speed = 0.5

		self.cabinet_level_arm_joints = [0.3750506556533743, 1.365694717508338, -0.4023128683165895, -0.800683612196537, -0.0010291761330924667, 3.4822743938715988, -0.5678680849764766]
		self.attack_arm_joints = [0.6759720515464899, -0.9762374636142142, -0.3832188963630161, -2.5237772660146436, -0.17070514597781364, 2.998545341354475, -1.0500396473031615]
		self.morph_resting_joints = [-1.6310335122148831, 0.16820470825782938, 0.31188980687764095, -2.714777928989822, -0.4470907467009208, 0.035748095159503876, -0.08582534693717468]
		self.morph_cabinet_base_pose = ((-1.7592102373614105, 1.3455894400761355, -1.4779547603947603), (-0.0007125657738850433, 0.00018374131576973096, 0.998870400542496, 0.0475119080780837))
		self.top_drawer_handle_close_pose =  [[-3.071478064394529, 1.2125335542071334, -0.6886245480738155] ,[0.625519052362346, -0.4671916992625666, -0.4602199529232834, 0.42267650301756654]]
		self.bottom_drawer_handle_close_pose =  [[-3.031478064394529, 1.2925335542071334, -0.9286245480738155] ,[0.625519052362346, -0.4671916992625666, -0.4602199529232834, 0.42267650301756654]]
		self.morph_init_open_drawer_joints = [1.8261201630826278, 1.7257997526868105, -0.6630539736023232, -1.229559703623374, -0.3520992647197664, 3.3392093252690516, -1.9661317900665047]

		self.top_locker_arm_joints = [-1.6603029407806946, 0.9231074757039127, 0.2674790666049998, -0.03740912180148676, 0.06095738792995047, 2.8188968646339676, 0.6099297548914087]
		self.top_locker_handle_pose = [[-2.967224182048576, 1.181980717529208, 0.0062629991445975735],[0.0034945356414816105, -0.5926434723374485, 0.084282809762041, 0.8010355241155526]]
		self.top_locker_base_pose = ((-2.21882054207176, 1.1780421114544755, -1.4781753440262075), (0.00019540959389212034, -0.00018999043907154625, -0.6772010393383353, 0.7357979872477243))

		self.left_top_drawer_arm_joints = [0.3893111886122389, 1.044664694460621, -0.3832188852980879, -1.1648994764536031, 0.028281842950589758, 3.3369946431002804, -0.9294345101239323]
		self.left_top_drawer_handle_pose = [[-2.778255993003974, -0.6411875356143112, -0.6040227059744339], [0.6736627051049497, -0.48442037803677185, -0.49719083839337996, 0.25360742755503163]]
		self.left_top_drawer_base_pose = ((-1.7734329332997985, -0.36829967030264366, -1.4779097661944167), (-0.00068536400658598, -0.0002574711331499127, 0.9952121200693554, 0.09773586880901446))

		self.stove_robot_base_pose = ((-2.093645434908393, 0.6831989779136299, -1.4782375377804404), (-0.0003549067901624782, 0.00037238479383245573, 0.9991095559243969, 0.04218803895999372))
		self.cube_top_grasp_pose = [[-2.890914937543639, 0.6432255037639979, -0.5228817490061057], [0.01845323118676989, 0.9064334589537164, 0.06001127516216436, -0.4176559703887638]]
		self.stove_object_joint_pose=[0.4854295582517043, 0.4956185068202427, -0.4138767235977286, -1.691198265522257, -0.16441308500765175, 2.998740491696482, 1.097690063790488, 0.0]

		# self.move_arm_to_attack_pose()

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
		# self.run_pick_food_from_stove_test()
		# self.rest_arm()
		# self.run_open_left_top_drawer_test()
		# self.run_open_bottom_drawer_test()
		# self.run_open_top_drawer_test()
		# self.run_close_top_drawer_test()
		self.arm_teleop()
		# for i in range(p.getNumJoints(self.morph)):
		# 	print(p.getJointInfo(self.morph,i))
		# self.move_arm_to_cabinet_level()
		# self.run_gripper_test()
		# self.set_gripper_to(0.02)
		# self.at_cabinet_pose = ((-2.1500243687136127, 1.092980084593978, -1.4781747553214735), 
		# 	(5.966205995056604e-05, 0.0009687922557074192, 0.7333478024202129, 0.6798529683449563))
		# self.move_base_to_position(2,-3,1.57)
		# self.run_draw_circle_test()
		# self.move_base_to_position(self.at_cabinet_pose[0][0],self.at_cabinet_pose[0][1],
		# 					1.57)
		time.sleep(60)

	def setup_environment(self):
		tableId = p.loadURDF("objects/table/table.urdf",[1.6087, -4.4277, -1.477],p.getQuaternionFromEuler([0,0,0]))
		fanta = Grocery_item(urdf_path='objects/can_sprite/sprite.obj',
			object_name='can_sprite', height=0.17,width=0.08,orr=1.57, urdf=False, p=p, x=-2.9, y=0.6,z=-0.5524)
		chair = p.loadSDF("objects/chair_1/model.sdf")
		p.resetBasePositionAndOrientation(chair[0], \
			[1.52078, -5.408, -1.4783], p.getQuaternionFromEuler([0,0,1.57]))

	def calculate_inverse_kinematics(self, targetPos,targetOr, threshold, maxIter):
		closeEnough = False
		iter = 0
		dist2 = 1e30
		while (not closeEnough and iter < maxIter):
			jointPoses = p.calculateInverseKinematics(self.morph, self.morph_end_effector, targetPos,targetOr)
			# for i in range(self.num_joints):
			# 	p.resetJointState(self.morph, i, jointPoses[i])
			ls = p.getLinkState(self.morph, self.morph_end_effector)
			newPos = ls[4]
			diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
			dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
			closeEnough = (dist2 < threshold)
			iter = iter + 1

		return jointPoses


	def move_arm_to_cabinet_level(self):
		joints = [0,1,2,3,4,5,6]
		p.setJointMotorControlArray(self.morph, joints, controlMode=p.POSITION_CONTROL,
									targetPositions=self.cabinet_arm_level)
		p.stepSimulation()


	def close_gripper(self):
		p.setJointMotorControl2(self.morph, self.morph_fingers_index[0],controlMode=p.POSITION_CONTROL,targetPosition=0.06)
		p.setJointMotorControl2(self.morph, self.morph_fingers_index[1],controlMode=p.POSITION_CONTROL,targetPosition=-0.06)
		p.stepSimulation()


	def open_gripper(self):
		p.setJointMotorControl2(self.morph, self.morph_fingers_index[0],controlMode=p.POSITION_CONTROL,targetPosition=0)
		p.setJointMotorControl2(self.morph, self.morph_fingers_index[1],controlMode=p.POSITION_CONTROL,targetPosition=0)
		p.stepSimulation()

	def set_gripper_to(self,value):
		p.setJointMotorControl2(self.morph, self.morph_fingers_index[0],controlMode=p.POSITION_CONTROL,targetPosition=value)
		p.setJointMotorControl2(self.morph, self.morph_fingers_index[1],controlMode=p.POSITION_CONTROL,targetPosition=value)
		p.stepSimulation()

	def rest_arm(self):
		time.sleep(2)
		joints = [0,1,2,3,4,5,6]
		for i in range(len(joints)):
			p.setJointMotorControl2(self.morph, joints[i], controlMode=p.POSITION_CONTROL,
										targetPosition=self.morph_resting_joints[i],
										velocityGain=1, positionGain=0.005)
			# time.sleep(1)
			p.stepSimulation()

	def create_object(self):
		position=[-2.9, 0.6,-0.5524]
		vid = p.createVisualShape(shapeType=p.GEOM_BOX, 
								   halfExtents=[.025,.025,.025],
								   rgbaColor=[1.,0.,1.,1])
		collisionShapeId = p.createCollisionShape(
								shapeType=p.GEOM_BOX,
		                        halfExtents=[.025,.025,.025])
		idd = p.createMultiBody(
					baseCollisionShapeIndex=collisionShapeId,
					baseVisualShapeIndex=vid,
					basePosition=position,
					# baseOrientation=orientation,
					baseMass=0.1
					)
		return idd

	def move_arm_to_attack_pose(self):
		time.sleep(2)
		joints = [0,1,2,3,4,5,6]
		for i in range(len(joints)):
			p.setJointMotorControl2(self.morph, joints[i], controlMode=p.POSITION_CONTROL,
										targetPosition=self.attack_arm_joints[i],
										# targetPosition=self.cabinet_level_arm_joints[i],
										velocityGain=1, positionGain=0.005)
			# time.sleep(1)
			p.stepSimulation()

	def move_arm_to_pose(self, position, orientation):
		angles = self.calculate_inverse_kinematics(position, orientation,
			threshold=0.0001, maxIter=1000)
		joints=[0,1,2,3,4,5,6]
		# js = self.left_top_drawer_arm_joints 
		js = angles[:7]
		for i in range(len(joints)):
			p.setJointMotorControl2(self.morph, joints[i], controlMode=p.POSITION_CONTROL,
									velocityGain=1, positionGain=0.005, targetPosition=js[i])
			p.stepSimulation()


	def orient_base_to_yaw(self, theta, tolerance=0.1, intelligent=True):
		pose, orientation = p.getBasePositionAndOrientation(self.morph)
		yaw = p.getEulerFromQuaternion(orientation)[2]
		wheelVelocities = [0, 0, 0, 0]
		wheelDeltasTurn = [1, -1, 1, -1]
		wheelDeltasFwd = [1, 1, 1, 1]
		print('THETA: ',theta)

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
					p.setJointMotorControl2(self.morph,
											self.wheels[i],
											p.VELOCITY_CONTROL,
											targetVelocity=wheelVelocities[i],
											force=1000)
				time.sleep(1)
				pose, orientation = p.getBasePositionAndOrientation(self.morph)
				yaw = p.getEulerFromQuaternion(orientation)[2]
				print('turning: ',yaw)
		else:
			if np.abs(theta) > 1.57 and np.abs(yaw) > 1.57:
				while np.abs(yaw - theta) > tolerance:
					wheelVelocities = [0, 0, 0, 0]
					if yaw < theta and (yaw > 0 and theta > 0):
						#turn anti-clockwise
						for i in range(len(self.wheels)):
							wheelVelocities[i] = wheelVelocities[i] - self.turn_speed * wheelDeltasTurn[i]
					elif yaw > theta and (yaw > 0 and theta > 0):
						#turn clockwise
						for i in range(len(self.wheels)):
							wheelVelocities[i] = wheelVelocities[i] + self.turn_speed * wheelDeltasTurn[i]
					elif yaw < theta and (yaw < 0 and theta < 0):
						#turn anti-clockwise
						for i in range(len(self.wheels)):
							wheelVelocities[i] = wheelVelocities[i] - self.turn_speed * wheelDeltasTurn[i]
					elif yaw > theta and (yaw > 0 and theta > 0):
						#turn clockwise
						for i in range(len(self.wheels)):
							wheelVelocities[i] = wheelVelocities[i] + self.turn_speed * wheelDeltasTurn[i]
					elif yaw < theta and (yaw < 0 and theta > 0):
						#turn clockwise
						for i in range(len(self.wheels)):
							wheelVelocities[i] = wheelVelocities[i] + self.turn_speed * wheelDeltasTurn[i]
					elif yaw > theta and (yaw > 0 and theta < 0):
						#turn anti-clockwise
						for i in range(len(self.wheels)):
							wheelVelocities[i] = wheelVelocities[i] - self.turn_speed * wheelDeltasTurn[i]

					else:
						#turn anti-clockwise
						for i in range(len(self.wheels)):
							wheelVelocities[i] = wheelVelocities[i] - self.turn_speed * wheelDeltasTurn[i]
					for i in range(len(self.wheels)):
						p.setJointMotorControl2(self.morph,
												self.wheels[i],
												p.VELOCITY_CONTROL,
												targetVelocity=wheelVelocities[i],
												force=1000)
					time.sleep(1)
					pose, orientation = p.getBasePositionAndOrientation(self.morph)
					yaw = p.getEulerFromQuaternion(orientation)[2]
					print('turning: ',yaw)
			elif np.abs(theta) <= 1.57 and np.abs(yaw) <= 1.57:
				while np.abs(yaw - theta) > tolerance:
					wheelVelocities = [0, 0, 0, 0]
					if yaw < theta and (yaw > 0 and theta > 0):
						#turn anti-clockwise
						for i in range(len(self.wheels)):
							wheelVelocities[i] = wheelVelocities[i] - self.turn_speed * wheelDeltasTurn[i]
					elif yaw > theta and (yaw > 0 and theta > 0):
						#turn clockwise
						for i in range(len(self.wheels)):
							wheelVelocities[i] = wheelVelocities[i] + self.turn_speed * wheelDeltasTurn[i]
					elif yaw < theta and (yaw < 0 and theta < 0):
						#turn anti-clockwise
						for i in range(len(self.wheels)):
							wheelVelocities[i] = wheelVelocities[i] - self.turn_speed * wheelDeltasTurn[i]
					elif yaw > theta and (yaw > 0 and theta > 0):
						#turn clockwise
						for i in range(len(self.wheels)):
							wheelVelocities[i] = wheelVelocities[i] + self.turn_speed * wheelDeltasTurn[i]
					elif yaw < theta and (yaw < 0 and theta > 0):
						#turn anti-clockwise
						for i in range(len(self.wheels)):
							wheelVelocities[i] = wheelVelocities[i] - self.turn_speed * wheelDeltasTurn[i]
					elif yaw > theta and (yaw > 0 and theta < 0):
						#turn clockwise
						for i in range(len(self.wheels)):
							wheelVelocities[i] = wheelVelocities[i] + self.turn_speed * wheelDeltasTurn[i]

					else:
						#turn anti-clockwise
						for i in range(len(self.wheels)):
							wheelVelocities[i] = wheelVelocities[i] - self.turn_speed * wheelDeltasTurn[i]
					for i in range(len(self.wheels)):
						p.setJointMotorControl2(self.morph,
												self.wheels[i],
												p.VELOCITY_CONTROL,
												targetVelocity=wheelVelocities[i],
												force=1000)
					time.sleep(1)
					pose, orientation = p.getBasePositionAndOrientation(self.morph)
					yaw = p.getEulerFromQuaternion(orientation)[2]
					print('turning: ',yaw)
			else:
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
						p.setJointMotorControl2(self.morph,
												self.wheels[i],
												p.VELOCITY_CONTROL,
												targetVelocity=wheelVelocities[i],
												force=1000)
					time.sleep(1)
					pose, orientation = p.getBasePositionAndOrientation(self.morph)
					yaw = p.getEulerFromQuaternion(orientation)[2]
					print('turning: ',yaw)

		print('done yawing')
		for i in range(len(self.wheels)):
				p.setJointMotorControl2(self.morph,
										self.wheels[i],
										p.VELOCITY_CONTROL,
										targetVelocity=0.0,
										force=1000)
		


	def drive_base_for_distance(self, goal_distance, tolerance=0.001):
		init_pos, orientation = p.getBasePositionAndOrientation(self.morph)
		
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
				p.setJointMotorControl2(self.morph,
										self.wheels[i],
										p.VELOCITY_CONTROL,
										targetVelocity=wheelVelocities[i],
										force=1000)

			pose, orientation = p.getBasePositionAndOrientation(self.morph)
			dist_covered = np.sqrt((init_pos[0]-pose[0])**2 + (init_pos[1]-pose[1])**2)
			print('linear: ',dist_covered)
		print('done translating')
		for i in range(len(self.wheels)):
				p.setJointMotorControl2(self.morph,
										self.wheels[i],
										p.VELOCITY_CONTROL,
										targetVelocity=0.0,
										force=1000)


	def move_base_to_position(self, gx, gy, theta):
		init_pos, orientation = p.getBasePositionAndOrientation(self.morph)
		direction = np.arctan2((gy-init_pos[1]), (gx-init_pos[0]))
		self.orient_base_to_yaw(direction)
		time.sleep(1)
		distance = np.sqrt((gx-init_pos[0])**2 + (gy-init_pos[1])**2)
		self.drive_base_for_distance(distance)
		time.sleep(1)
		self.orient_base_to_yaw(theta)

		# init_pos, orientation = p.getBasePositionAndOrientation(self.morph)
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

	def stow_gripper(self):
		self.control_joint(14, -3, -3.14,3.14)

	def ready_gripper(self):
		self.control_joint(14, 0, -3.14,3.14)

	def twist_gripper_vertical(self):
		self.control_joint(15, 1.57, -3.14,3.14)

	def twist_gripper_horizontal(self):
		self.control_joint(15, 0, -3.14,3.14)

	def control_joint(self, joint, value, minn, maxx):
		if value < minn:
			value = minn
		if value > maxx:
			value = maxx
		p.setJointMotorControl2(self.morph, joint,
					controlMode=p.POSITION_CONTROL,targetPosition=value,
					velocityGain=1, positionGain=0.005)
		p.stepSimulation()


	def extend_arm_to(self,dist):
		min_limit = 0; max_limit=0.25
		joints_from_ee = [13,12,11,10]
		if dist <= 0.3:
			return False
		if dist <= 0.55:
			self.control_joint(joints_from_ee[0],
				(dist-0.3),min_limit, max_limit)
			self.control_joint(joints_from_ee[1],
				0,min_limit, max_limit)
			self.control_joint(joints_from_ee[2],
				0,min_limit, max_limit)
			self.control_joint(joints_from_ee[3],
				0,min_limit, max_limit)

		elif dist > 0.55 and dist <= 0.8:
			self.control_joint(joints_from_ee[0],
				0.25,min_limit, max_limit)
			self.control_joint(joints_from_ee[1],
				(dist-0.55),min_limit, max_limit)
			self.control_joint(joints_from_ee[2],
				0,min_limit, max_limit)
			self.control_joint(joints_from_ee[3],
				0,min_limit, max_limit)

		elif dist > 0.8 and dist <= 1.05:
			self.control_joint(joints_from_ee[0],
				0.25,min_limit, max_limit)
			self.control_joint(joints_from_ee[1],
				0.25,min_limit, max_limit)
			self.control_joint(joints_from_ee[2],
				(dist-0.8),min_limit, max_limit)
			self.control_joint(joints_from_ee[3],
				0,min_limit, max_limit)

		elif dist > 1.05 and dist <= 1.3:
			self.control_joint(joints_from_ee[0],
				0.25,min_limit, max_limit)
			self.control_joint(joints_from_ee[1],
				0.25,min_limit, max_limit)
			self.control_joint(joints_from_ee[2],
				0.25,min_limit, max_limit)
			self.control_joint(joints_from_ee[3],
				(dist-1.05),min_limit, max_limit)
		elif dist > 1.3:
			return False
		return True


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
		dist = 0.3
		height = 0.0
		while 1:
			wheelVelocities = [0, 0, 0, 0]
			keys = p.getKeyboardEvents()
			if ord('u') in keys:
				ji = 9
				height = p.getJointState(self.morph, ji)[0]
				if height < 1.3:
					height += 0.001
				self.control_joint(ji, height, 0.0, 1.3)

			if ord('i') in keys:
				ji = 9
				height = p.getJointState(self.morph, ji)[0]
				if height > 0.0:
					height -= 0.001
				self.control_joint(ji, height, 0.0,1.3)

			if ord('j') in keys:
				if dist < 1.4:
					dist += 0.001
				self.extend_arm_to(dist)

			if ord('k') in keys:
				if dist > 0.3:
					dist -= 0.001
				self.extend_arm_to(dist)

			if ord('n') in  keys:
				ji = 14
				currjoint = p.getJointState(self.morph, ji)[0]
				self.control_joint(ji, currjoint+0.01,-3.14,3.14)

			if ord('m') in  keys:
				ji = 14
				currjoint = p.getJointState(self.morph, ji)[0]
				self.control_joint(ji, currjoint-0.01,-3.14,3.14)

			if ord('o') in  keys:
				self.twist_gripper_vertical()
				# # ji = 15
				# currjoint = p.getJointState(self.morph, ji)[0]
				# self.control_joint(ji, currjoint+0.01,-3.14,3.14)

			if ord('p') in  keys:
				self.twist_gripper_horizontal()
				# ji = 15
				# currjoint = p.getJointState(self.morph, ji)[0]
				# self.control_joint(ji, currjoint-0.01,-3.14,3.14)

			if ord('0') in keys:
				self.move_arm_to_pose(self.top_drawer_handle_close_pose[0], 
					self.top_drawer_handle_close_pose[1])

			if ord('1') in keys:
				self.close_gripper()

			if ord('2') in keys:
				self.open_gripper()
				
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
				p.setJointMotorControl2(self.morph,
									wheels[i],
									p.VELOCITY_CONTROL,
									targetVelocity=wheelVelocities[i],
									force=1000)
			# p.stepSimulation()

			joint_angles=[]
			
			pose = p.getBasePositionAndOrientation(self.morph)
			print('BASE POSE AND ORIENTATION: ',pose)
			print('ARM HEIGHT: ',height)
			print('ARM LENGTH: ',dist)
			print('&'*30)
			print(' ')

	def run_pick_food_from_stove_test(self):
		self.stow_gripper()
		basepose = ((-2.100468710096393, 0.9231578949558297, -1.477370604548337), (-0.0008581358239357019, -0.0005171708162984435, -0.7119393288519943, 0.702240263849223))
		theta = p.getEulerFromQuaternion(basepose[1])[2]
		self.move_base_to_position(basepose[0][0], basepose[0][1],theta)
		time.sleep(1)

		height = 0.406
		length = 0.6
		self.control_joint(9, height, 0.0, 1.3)
		time.sleep(3)
		self.open_gripper()
		time.sleep(3)
		self.ready_gripper()
		time.sleep(3)
		self.extend_arm_to(length)
		time.sleep(3)
		self.close_gripper()
		time.sleep(3)
		self.control_joint(9, height+0.2, 0.0, 1.3)
		self.extend_arm_to(length-0.4)
		time.sleep(3)

		destpose = ((1.6820733918269681, -3.2852051873822616, -1.4771831169755152), (-6.802841669425193e-05, -0.0008524114072207692, -0.030760320506220924, 0.9995264235873317))
		theta = p.getEulerFromQuaternion(destpose[1])[2]
		self.move_base_to_position(destpose[0][0], destpose[0][1],theta)
		time.sleep(1)

		height = 0.3
		length = 1.1
		self.control_joint(9, height, 0.0, 1.3)
		time.sleep(3)
		self.extend_arm_to(length)
		time.sleep(3)
		self.open_gripper()
		time.sleep(3)
		self.control_joint(9, height+0.4, 0.0, 1.3)
		time.sleep(3)
		self.extend_arm_to(0.4)
		time.sleep(3)
		self.stow_gripper()
		time.sleep(3)
		






	def run_open_top_drawer_test(self):
		self.open_gripper()
		theta = p.getEulerFromQuaternion(self.morph_cabinet_base_pose[1])[2]
		self.move_base_to_position(self.morph_cabinet_base_pose[0][0],
			self.morph_cabinet_base_pose[0][1],theta)
		time.sleep(1)
		# self.drive_base_for_distance(-0.2)
		self.move_arm_to_pose(self.top_drawer_handle_close_pose[0],
								self.top_drawer_handle_close_pose[1])
		time.sleep(10)
		self.drive_base_for_distance(0.0199)
		time.sleep(5)
		self.close_gripper()
		time.sleep(5)
		self.drive_base_for_distance(-0.5)
		time.sleep(1)
		self.open_gripper()
		time.sleep(1)
		# self.rest_arm()
		p.stepSimulation()

	def run_close_top_drawer_test(self):
		self.close_gripper()
		self.move_arm_to_pose(self.top_drawer_handle_close_pose[0],
								self.top_drawer_handle_close_pose[1])
		time.sleep(3)
		theta = p.getEulerFromQuaternion(self.morph_cabinet_base_pose[1])[2]
		self.move_base_to_position(self.morph_cabinet_base_pose[0][0]-0.1,
			self.morph_cabinet_base_pose[0][1],theta)
		
		time.sleep(5)
		self.drive_base_for_distance(-0.5)
		self.open_gripper()
		p.stepSimulation()

	def run_open_bottom_drawer_test(self):
		self.open_gripper()
		theta = p.getEulerFromQuaternion(self.morph_cabinet_base_pose[1])[2]
		self.move_base_to_position(self.morph_cabinet_base_pose[0][0],
			self.morph_cabinet_base_pose[0][1],theta)
		time.sleep(1)
		# self.drive_base_for_distance(-0.2)
		self.move_arm_to_pose(self.bottom_drawer_handle_close_pose[0],
								self.bottom_drawer_handle_close_pose[1])
		time.sleep(10)
		self.drive_base_for_distance(0.055)
		time.sleep(5)
		self.close_gripper()
		time.sleep(5)
		self.drive_base_for_distance(-0.5)
		time.sleep(1)
		self.open_gripper()
		time.sleep(1)
		# self.rest_arm()
		p.stepSimulation()

	def run_close_bottom_drawer_test(self):
		self.close_gripper()
		self.move_arm_to_pose(self.bottom_drawer_handle_close_pose[0],
								self.bottom_drawer_handle_close_pose[1])
		time.sleep(3)
		theta = p.getEulerFromQuaternion(self.morph_cabinet_base_pose[1])[2]
		self.move_base_to_position(self.morph_cabinet_base_pose[0][0]-0.1,
			self.morph_cabinet_base_pose[0][1],theta)
		
		time.sleep(5)
		self.drive_base_for_distance(-0.5)
		self.open_gripper()
		p.stepSimulation()

	def run_open_left_top_drawer_test(self):
		self.open_gripper()
		theta = p.getEulerFromQuaternion(self.left_top_drawer_base_pose[1])[2]
		self.move_base_to_position(self.left_top_drawer_base_pose[0][0],
			self.left_top_drawer_base_pose[0][1],theta)
		time.sleep(1)
		self.move_arm_to_pose(self.left_top_drawer_handle_pose[0],
								self.left_top_drawer_handle_pose[1])
		time.sleep(10)
		self.drive_base_for_distance(0.055)
		time.sleep(5)
		self.close_gripper()
		time.sleep(5)
		self.drive_base_for_distance(-0.5)
		time.sleep(1)
		self.open_gripper()
		time.sleep(1)
		# self.rest_arm()
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
				p.setJointMotorControl2(self.morph,
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
			# 	p.resetJointState(self.morph, i, jointPoses[i])
			# ls = p.getLinkState(self.morph, self.morph_end_effector)
			# if (hasPrevPose):
			# 	p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
			# 	p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
			# prevPose = pos
			# prevPose1 = ls[4]
			# hasPrevPose = 1
			print(p.getBasePositionAndOrientation(self.morph))
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