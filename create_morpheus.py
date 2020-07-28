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

floor = p.loadURDF('floor/floor.urdf',useFixedBase=True)
morph = p.loadURDF('morpheus_description/morph_with_horizontal_gripper.urdf')#, [0,0,0], p.getQuaternionFromEuler((0,0,0)))

# husky = p.loadURDF("husky/husky.urdf", [0,0,0])
# for i in range(p.getNumJoints(morph)):
#   print(p.getJointInfo(morph, i))
# cid = p.createConstraint(husky, 7, morph, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0., 0., 5],
                         # [0, 0, 0, 1])

# p.resetJointState(morph, 0, 1.3)
# for t in np.arange(0, 1.3, 0.05):
# 	p.resetJointState(morph, 0, t)
# 	time.sleep(1)
# print('done')
_link_name_to_index = {p.getBodyInfo(morph)[0].decode('UTF-8'):-1,}
        
for _id in range(p.getNumJoints(morph)):
	_name = p.getJointInfo(morph, _id)[12].decode('UTF-8')
	_link_name_to_index[_name] = _id
print(_link_name_to_index)
time.sleep(300)