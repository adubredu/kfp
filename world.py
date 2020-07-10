from __future__ import print_function
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tools'))

import numpy as np
import time
import pybullet  as p
import pybullet_data
import utils as ut
import ikfast.franka_panda.ik as fik


class World:
	def __init__(self):
		self.client = p.connect(p.GUI)
		p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
		p.setAdditionalSearchPath('models')
		kitchen_path = 'kitchen_description/urdf/kitchen_part_right_gen_convex.urdf'

		with ut.HideOutput(enable=True):
			self.floor = p.loadURDF('floor/floor.urdf',useFixedBase=True)
			self.kitchen = p.loadURDF(kitchen_path,useFixedBase=True)

		z = ut.stable_z(self.kitchen, self.floor) - ut.get_point(self.floor)[2]
		point = np.array(ut.get_point(self.kitchen)) - np.array([0, 0, z])
		ut.set_point(self.floor, point)

		franka_path = 'panda_arm_hand_on_carter_visual.urdf'

		self.robot_name = "franka_carter"
		with ut.HideOutput(enable=True):
			self.robot = p.loadURDF(franka_path, (2,2,0))

		ut.set_point(self.robot, ut.Point(z=ut.stable_z(self.robot,self.floor)))
		self.set_initial_conf()
		self.gripper = ut.create_gripper(self.robot)

		#######################################
		self.environment_bodies = {}
		# if full_kitchen:
		self._initialize_environment()
		self.initial_saver = ut.WorldSaver()

		self.body_from_name = {}
		# self.path_from_name = {}
		self.names_from_type = {}
		self.custom_limits = {}
		self.base_limits_handles = []
		self.cameras = {}

		self.door_ids = {
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

		self.disabled_collisions = set()
		if self.robot_name == 'franka_carter':
			self.disabled_collisions.update(tuple(ut.link_from_name(self.robot, link) for link in pair)
											for pair in ut.DISABLED_FRANKA_COLLISIONS)

		self.carry_conf = ut.FConf(self.robot, self.arm_joints, self.default_conf)
		#self.calibrate_conf = Conf(self.robot, self.arm_joints, load_calibrate_conf(side='left'))
		self.calibrate_conf = ut.FConf(self.robot, self.arm_joints, self.default_conf) # Must differ from carry_conf
		self.special_confs = [self.carry_conf] #, self.calibrate_conf]
		self.open_gq = ut.FConf(self.robot, self.gripper_joints,
							ut.get_max_limits(self.robot, self.gripper_joints))
		self.closed_gq = ut.FConf(self.robot, self.gripper_joints,
							  ut.get_min_limits(self.robot, self.gripper_joints))
		self.gripper_confs = [self.open_gq, self.closed_gq]
		self.open_kitchen_confs = {joint: ut.FConf(self.kitchen, [joint], [self.open_conf(joint)])
								   for joint in self.kitchen_joints}
		self.closed_kitchen_confs = {joint: ut.FConf(self.kitchen, [joint], [self.closed_conf(joint)])
									 for joint in self.kitchen_joints}
		# print(self.arm_joints)
		print(self.tool_link)#23
		print(self.gripper_link)#20
		print(self.franka_link)#11
		# self._update_custom_limits()
		# self._update_initial()
		self.ik_test()
		# jointPositions = [3.559609, 0.411182, 0.862129, 1.744441, 0.077299, -1.129685, 0.006001]
		# print('joints: ',p.getNumJoints(self.robot))
		# for jointIndex in range(len(self.arm_joints)):
		# 	time.sleep(2)
		# 	p.resetJointState(self.robot, self.arm_joints[jointIndex], jointPositions[jointIndex])

	def open_door_with_name(self, name):
		index = self.door_ids[name]
		self.open_door(index)


	def close_door_with_name(self, name):
		index = self.door_ids[name]
		self.close_door(index)


	def ik_test(self):
		t=0
		# jointangles = self.solve_inverse_kinematics([[2,2,1], p.getQuaternionFromEuler([0, -np.pi, 0])])
		
		while 1:
			t += 0.001
			pos = [2+0.8, 0.2 * np.cos(t)+2, 0. + 0.2 * np.sin(t) + 1.3]
			orn = p.getQuaternionFromEuler([0, -np.pi, 0])
			pos = [2,2,1.3]
			pp = (pos,orn)
			
			# print('franka id: ',self.franka_link)
			# print('ee id: ',self.gripper_link)
			jointangles = self.solve_inverse_kinematics(pp)
			print(jointangles)

			# for i in range(len(self.arm_joints)):
			# 	p.resetJointState(self.robot, self.arm_joints[i], jointangles[i])



		print('DONE!!!!')




	def _initialize_environment(self):
		# wall to fridge: 4cm
		# fridge to goal: 1.5cm
		# hitman to range: 3.5cm
		# range to indigo: 3.5cm
		poses_path = 'models/kitchen_poses.json'
		self.environment_poses = ut.read_json(poses_path)
		root_from_world = ut.get_link_pose(self.kitchen, self.world_link)
		for name, world_from_part in self.environment_poses.items():
			if name in ['range']:
				continue
			visual_path = os.path.join(ut.KITCHEN_LEFT_PATH, '{}.obj'.format(name))
			collision_path = os.path.join(ut.KITCHEN_LEFT_PATH, '{}_collision.obj'.format(name))
			mesh_path = None
			for path in [collision_path, visual_path]:
				if os.path.exists(path):
					mesh_path = path
					break
			if mesh_path is None:
				continue
			body = ut.load_pybullet(mesh_path, fixed_base=True)
			root_from_part = ut.multiply(root_from_world, world_from_part)
			if name in ['axe', 'dishwasher', 'echo', 'fox', 'golf']:
				(pos, quat) = root_from_part
				pos = np.array(pos) + np.array([0, -0.035, 0])  
				root_from_part = (pos, quat)
			self.environment_bodies[name] = body
			ut.set_pose(body, root_from_part)

		if ut.TABLE_NAME in self.environment_bodies:
			body = self.environment_bodies[ut.TABLE_NAME]
			_, (w, l, _) = approximate_as_prism(body)
			_, _, z = get_point(body)
			new_pose = Pose(Point(TABLE_X + l / 2, -TABLE_Y, z), Euler(yaw=np.pi / 2))
			set_pose(body, new_pose)


	def get_base_conf(self):
		return ut.get_joint_positions(self.robot, self.base_joints)
	def set_base_conf(self, conf):
		ut.set_joint_positions(self.robot, self.base_joints, conf)
	def get_base_aabb(self):
		franka_links = ut.get_link_subtree(self.robot, self.franka_link)
		base_links = ut.get_link_subtree(self.robot, self.base_link)
		return ut.aabb_union(ut.get_aabb(self.robot, link) for link in set(base_links) - set(franka_links))
	def get_world_aabb(self):
		return ut.aabb_union(ut.get_aabb(body) for body in self.fixed)


	def _update_custom_limits(self, min_extent=0.0):
		#robot_extent = get_aabb_extent(get_aabb(self.robot))
		# Scaling by 0.5 to prevent getting caught in corners
		#min_extent = 0.5 * min(robot_extent[:2]) * np.ones(2) / 2
		world_aabb = self.get_world_aabb()
		full_lower, full_upper = world_aabb
		base_limits = (full_lower[:2] - min_extent, full_upper[:2] + min_extent)
		base_limits[1][0] = 2.40 - min_extent # TODO: robot radius
		base_limits[0][1] += 0.1
		base_limits[1][1] -= 0.1
		for handle in self.base_limits_handles:
			ut.remove_debug(handle)
		self.base_limits_handles = []
		#self.base_limits_handles.extend(draw_aabb(world_aabb))
		z = ut.get_point(self.floor)[2] + 1e-2
		self.base_limits_handles.extend(ut.draw_base_limits(base_limits, z=z))
		self.custom_limits = ut.custom_limits_from_base_limits(self.robot, base_limits)
		return self.custom_limits


	def _update_initial(self):
		# TODO: store initial poses as well?
		self.initial_saver = ut.WorldSaver()
		self.goal_bq = ut.FConf(self.robot, self.base_joints)
		self.goal_aq = ut.FConf(self.robot, self.arm_joints)
		if ut.are_confs_close(self.goal_aq, self.carry_conf):
			self.goal_aq = self.carry_conf
		self.goal_gq = ut.FConf(self.robot, self.gripper_joints)
		self.initial_confs = [self.goal_bq, self.goal_aq, self.goal_gq]
		ut.set_all_static()


	def set_initial_conf(self):
		ut.set_joint_positions(self.robot, self.base_joints, [2.0, 0, np.pi])
		ut.set_joint_positions(self.robot, self.arm_joints, self.default_conf)


	def set_gripper(self, value):
		positions = value*np.ones(len(self.gripper_joints))
		set_joint_positions(self.robot, self.gripper_joints, positions)
	

	def close_gripper(self):
		self.closed_gq.assign()
	

	def open_gripper(self):
		self.open_gq.assign()
	

	@property
	def constants(self):
		return self.special_confs + self.gripper_confs + self.initial_confs

	@property
	def base_joints(self):
		return ut.joints_from_names(self.robot, ut.BASE_JOINTS)
	@property
	def arm_joints(self):
		joint_names = ['panda_joint{}'.format(1+i) for i in range(7)]
		return ut.joints_from_names(self.robot, joint_names)
	@property
	def gripper_joints(self):
		joint_names = ['panda_finger_joint{}'.format(1+i) for i in range(2)]
		return ut.joints_from_names(self.robot, joint_names)

	@property
	def kitchen_joints(self):
		joint_names = ut.get_joint_names(self.kitchen, ut.get_movable_joints(self.kitchen))
		return ut.joints_from_names(self.kitchen, filter(ut.ALL_JOINTS.__contains__, joint_names))

	@property
	def base_link(self):
		return ut.child_link_from_joint(self.base_joints[-1])

	@property
	def franka_link(self):
		return ut.parent_link_from_joint(self.robot, self.arm_joints[0])

	@property
	def gripper_link(self):
		return ut.parent_link_from_joint(self.robot, self.gripper_joints[0])

	@property
	def tool_link(self):
		return ut.link_from_name(self.robot, ut.get_tool_link(self.robot))

	@property
	def world_link(self): # for kitchen
		return ut.BASE_LINK

	@property
	def door_links(self):
		door_links = set()
		for joint in self.kitchen_joints:
			door_links.update(ut.get_link_subtree(self.kitchen, joint))
		return door_links

	@property
	def static_obstacles(self):
		return {(self.kitchen, frozenset([link])) for link in
				set(get_links(self.kitchen)) - self.door_links} | \
			   {(body, None) for body in self.environment_bodies.values()}

	@property
	def movable(self): # movable base
		return set(self.body_from_name) # frozenset?

	@property
	def fixed(self): # fixed base
		return set(self.environment_bodies.values()) | {self.kitchen}

	@property
	def all_bodies(self):
		return self.movable | self.fixed | {self.robot}

	@property
	def default_conf(self):
		return ut.DEFAULT_ARM_CONF

	def get_door_sign(self, joint):
		return -1 if 'left' in ut.get_joint_name(self.kitchen, joint) else +1
	
	def closed_conf(self, joint):
		lower, upper = ut.get_joint_limits(self.kitchen, joint)
		if 'drawer' in ut.get_joint_name(self.kitchen, joint):
			fraction = 0.9
			return fraction*lower + (1-fraction)*upper
		if 'left' in ut.get_joint_name(self.kitchen, joint):
			return upper
		return lower
	
	def open_conf(self, joint):
		joint_name = ut.get_joint_name(self.kitchen, joint)
		if 'left' in joint_name:
			open_position = ut.get_min_limit(self.kitchen, joint)
		else:
			open_position = ut.get_max_limit(self.kitchen, joint)
		#print(get_joint_name(self.kitchen, joint), get_min_limit(self.kitchen, joint), get_max_limit(self.kitchen, joint))
		# drawers: [0.0, 0.4]
		# left doors: [-1.57, 0.0]
		# right doors: [0.0, 1.57]
		if joint_name in ut.CABINET_JOINTS:
			# TODO: could make fraction of max
			return ut.CABINET_OPEN_ANGLE * open_position / abs(open_position)
		if joint_name in ut.DRAWER_JOINTS:
			return ut.DRAWER_OPEN_FRACTION * open_position
		return open_position
	
	def close_door(self, joint):
		ut.set_joint_position(self.kitchen, joint, self.closed_conf(joint))
	
	def open_door(self, joint):
		ut.set_joint_position(self.kitchen, joint, self.open_conf(joint))

	def solve_inverse_kinematics(self, world_from_tool, nearby_tolerance=np.inf, **kwargs):
		current_conf = ut.get_joint_positions(self.robot, self.arm_joints)
		start_time = time.time()
		generator = fik.ikfast_inverse_kinematics(self.robot, fik.PANDA_INFO, self.tool_link, world_from_tool,
												  max_attempts=10, use_halton=True)
		conf = next(generator, None)
		#conf = sample_tool_ik(self.robot, world_from_tool, max_attempts=100)
		if conf is None:
			return conf
		max_distance = ut.get_distance(current_conf, conf, norm=np.inf)
		#print('Time: {:.3f} | distance: {:.5f} | max: {:.5f}'.format(
		#    elapsed_time(start_time), max_distance, nearby_tolerance))
		ut.set_joint_positions(self.robot, self.arm_joints, conf)
		return ut.get_configuration(self.robot)




if __name__ == '__main__':
	w = World()
	# for i in range(p.getNumJoints(w.kitchen)):
	# 	print(p.getJointInfo(w.kitchen,i))
	# while True:
	# 	w.open_gripper()
	# 	for i in w.doors.values():
	# 		w.open_door(i)
	# 	time.sleep(2)
	# 	w.close_gripper()
	# 	for i in w.doors.values():
	# 		w.close_door(i)
	# 	time.sleep(2)
	time.sleep(100)





