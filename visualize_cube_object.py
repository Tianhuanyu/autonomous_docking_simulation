import pybullet as pb

pb.connect(pb.GUI)
pb.resetSimulation()
pb.setAdditionalSearchPath('resources')

box_id = pb.loadURDF(
    "cube_hole.urdf",
    useMaximalCoordinates = True,
    useFixedBase=1,
)
pb.changeVisualShape(box_id, 0, rgbaColor=[124./255., 252./255., 0., 1])

while pb.isConnected():
    pass
