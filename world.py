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


if __name__ == '__main__':
	w = World()
	time.sleep(100)




