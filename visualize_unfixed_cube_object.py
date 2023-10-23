import pybullet as pb
import numpy as np
from time import sleep

pb.connect(pb.GUI, options='--mp4=videos/unfixed_cube.mp4')
pb.resetSimulation()
pb.setAdditionalSearchPath('resources')
pb.setGravity(gravX=0, gravY=0, gravZ=-9.81)

pb.setTimeStep(1./240.)

pb.resetDebugVisualizerCamera(
    cameraDistance=0.075,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0],
)

pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)

box_id = pb.loadURDF(
    "cube_hole.urdf",
    useMaximalCoordinates = True,
    globalScaling=0.03,
)

pb.changeVisualShape(box_id, 0, rgbaColor=[124./255., 252./255., 0., 1])

cloth_left_id = pb.loadSoftBody(
    "small_cloth.obj",
    basePosition = [-0.03, 0, 0],
    mass = 0.05,
    useNeoHookean = 0,
    useBendingSprings=1,
    useMassSpring=1,
    springElasticStiffness=5000,
    springDampingStiffness=400.,
    springDampingAllDirections = 1,
    useSelfCollision = 0,
    frictionCoeff = .5*0.03,
    useFaceContact=1
)

cloth_right_id = pb.loadSoftBody(
    "small_cloth.obj",
    basePosition = [0.03, 0, 0],
    mass = 0.05,
    useNeoHookean = 0,
    useBendingSprings=1,
    useMassSpring=1,
    springElasticStiffness=5000,
    springDampingStiffness=400.,
    springDampingAllDirections = 1,
    useSelfCollision = 0,
    frictionCoeff = .5*0.03,
    useFaceContact=1
)

pb.changeVisualShape(cloth_left_id, -1, flags=pb.VISUAL_SHAPE_DOUBLE_SIDED)
pb.changeVisualShape(cloth_right_id, -1, flags=pb.VISUAL_SHAPE_DOUBLE_SIDED)

pb.createSoftBodyAnchor(cloth_left_id, 19, -1, -1) # outer corner 1
pb.createSoftBodyAnchor(cloth_left_id, 20, -1, -1) # outer corner 2

pb.createSoftBodyAnchor(cloth_right_id, 15, -1, -1) # outer corner 1
pb.createSoftBodyAnchor(cloth_right_id, 24, -1, -1) # outer corner 2

pb.createSoftBodyAnchor(cloth_left_id, 15, box_id, -1, [-0.015, 0.015, 0])
pb.createSoftBodyAnchor(cloth_left_id, 24, box_id, -1, [-0.015,-0.015, 0])

pb.createSoftBodyAnchor(cloth_right_id, 19, box_id, -1, [0.015, 0.015, 0])
pb.createSoftBodyAnchor(cloth_right_id, 20, box_id, -1, [0.015, -0.015,0])

pb.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)

hole_initial_central_position = np.array([0, 0, 0])

cloth_left_base_position = hole_initial_central_position + np.array([-0.03, 0, 0])
cloth_right_base_position = hole_initial_central_position + np.array([0.03, 0, 0])
item_rotation = pb.getQuaternionFromEuler(np.deg2rad([-45, 0, 0]))

pb.resetBasePositionAndOrientation(cloth_left_id, cloth_left_base_position , item_rotation)
pb.resetBasePositionAndOrientation(cloth_right_id,  cloth_right_base_position, item_rotation)
pb.resetBasePositionAndOrientation(box_id, hole_initial_central_position, item_rotation)

while pb.isConnected():
    pb.stepSimulation()
    # sleep(1./240.)
    # sleep(100./240.)
    # sleep(1000./240.)

    sleep(10./240.)
