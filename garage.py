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

client = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setAdditionalSearchPath('models')

# vid = p.createVisualShape(shapeType=p.GEOM_MESH,
#                                     fileName="platform/platform.obj",
#                                     rgbaColor=[0,0,0, 1],
#                                     specularColor=[0.4, .4, 0],
#                                     visualFramePosition=shift,
#                                     meshScale=meshScale)
visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX,
									halfExtents=[0.3,0.3,0.3],
									rgbaColor=[0,0,0,1])
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX,
			                              halfExtents=[0.3,0.3,0.3])

p.createMultiBody(baseMass=1,
                      baseInertialFramePosition=[0, 0, 0],
                      baseCollisionShapeIndex=collisionShapeId,
                      baseVisualShapeIndex=visualShapeId,
                      basePosition=[0,0,1],
                      useMaximalCoordinates=True)

for i in range(1000):
	p.stepSimulation()
	time.sleep(1)