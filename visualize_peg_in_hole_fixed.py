import numpy as np
import pybullet as pb
from time import sleep, time

##########################################################
## Setup simulator

pb.connect(pb.GUI)
pb.resetSimulation()
pb.setAdditionalSearchPath('resources')

gravz = -9.81
pb.setGravity(0, 0, gravz)

sampling_freq = 240
time_step = 1./float(sampling_freq)
pb.setTimeStep(time_step)

box_base_position = np.array([0.35, -0.2, 0.2])

pb.resetDebugVisualizerCamera(
    cameraDistance=0.2,
    cameraYaw=-40,
    cameraPitch=30,
    cameraTargetPosition=box_base_position,
)
pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)

##########################################################
## Load environment

box_base_orientation = np.array(pb.getQuaternionFromEuler([45, 0, 0]))

box_id = pb.loadURDF(
    "cube_hole.urdf",
    baseOrientation=box_base_orientation,
    basePosition=box_base_position,
    useMaximalCoordinates = True,
    useFixedBase=1,
    globalScaling=0.03,
)
pb.changeVisualShape(box_id, 0, rgbaColor=[124./255., 252./255., 0., 1])

##########################################################
## Load robot

class Robot:

    def __init__(self):
        self.id = pb.loadURDF(
            'lbr_with_tilt_endoscope.urdf',
            basePosition=[1, 0, 0],
            baseOrientation=pb.getQuaternionFromEuler([0, 0, 0]),
        )

    @property
    def num_joints(self):
        return pb.getNumJoints(self.id)

    @property
    def joint_indices(self):
        return list(range(self.num_joints))

    @property
    def joint_info(self):
        joint_info = []
        for joint_index in self.joint_indices:
            joint_info.append(pb.getJointInfo(self.id, joint_index))
        return joint_info

    @property
    def joint_types(self):
        return [info[2] for info in self.joint_info]

    @property
    def revolute_joint_indices(self):
        return [
            joint_index
            for joint_index, joint_type in zip(self.joint_indices, self.joint_types)
            if joint_type == pb.JOINT_REVOLUTE
        ]

    @property
    def ndof(self):
        return len(self.revolute_joint_indices)

    def reset(self, q):
        for joint_index, joint_position in zip(self.revolute_joint_indices, q):
            pb.resetJointState(self.id, joint_index, joint_position)

    def cmd(self, q):
        pb.setJointMotorControlArray(
            self.id,
            self.revolute_joint_indices,
            pb.POSITION_CONTROL,
            q,
        )


robot = Robot()
q0 = np.deg2rad([-40, -45, 0, 90, 0, -45, 0])
robot.reset(q0)
robot.cmd(q0)

##########################################################
## Run simulation

while pb.isConnected():
    pb.stepSimulation()
    sleep(time_step)
