import optas
import sys
import numpy as np
import pybullet as pb
import matplotlib.pyplot as plt
from time import sleep, time, perf_counter, time_ns
from scipy.spatial.transform import Rotation as Rot
from optas.spatialmath import *

import scipy.signal as signal
# from scipy.spatial.transform import Rotation
import copy
import math


np.set_printoptions(precision=3, linewidth=1000, suppress=True)

##########################################################
## Setup planner

obstacle_radius = 0.01
endoscope_tip_radii = 0.0025

def skew_np(v):
    """! Create skew-symmetric matrix.

    If V (1x1) then (order 2) S =

          | 0  -v |
          | v   0 |

    and if V (1x3) then (order 3) S =

          |  0  -vz   vy |
          | vz    0  -vx |
          |-vy   vx    0 |

    @param v The form of the skew-symmetric matrix, either a scalar or vector with 3 elements.
    @return Skew-symmetric matrix of order 2 or 3.
    """
    if v.shape[0] == 1:
        return np.hstack([
            np.vstack([0.0, -v]),
            np.vstack([v, 0.0])
        ]
        )
    elif v.shape[0] == 3:
        return np.hstack([
            np.vstack([0.0, -v[2], v[1]]),
            np.vstack([v[2], 0.0, -v[0]]),
            np.vstack([-v[1], v[0], 0.0])]
        )
    else:
        raise ValueError(f"expecting a scalar or 3-vector")
    
def calculate_new_axis(theta, axis_last):

    r = Rot.from_euler('y', theta)
    R_y = r.as_matrix()

    axis_new = R_y @ axis_last.reshape(3,-1)

    return axis_new


def rotation_matrix_z(theta):

    r = Rot.from_euler('z', theta)
    R_z = r.as_matrix()
    
    return R_z

def find_xz_plane_RT(f,mf,Tr):
    # f_lin = f
    # # f_rot = m[3:]
    # f_rot = mf

    RT = np.zeros((4,4))
    RT[3,3] = 1.0

    Rr = Tr[:3,:3]

    fr = Rr.T @ f
    mr = Rr.T @ mf

    v_n = np.array([1.0, 0.0, 0.0])
    axisz = Rr[:,2]
    f_x = f - np.dot(f, axisz) * axisz
    # v = f_x
    v = copy.deepcopy(fr)
    v[2] = 0.0
    # v = Rr @ v

    # print("np.dot(f, axisz) * axisz = ",np.dot(f, axisz))
    # print("np.dot(f, axisz) * axisz = ",np.linalg.norm(f_x))

    # if(np.linalg.norm(f_x)<1e-3):

    

    if(np.linalg.norm(v) > 1e-6):
        # angle = np.arccos(np.dot(v, v_n) / (np.linalg.norm(v) * np.linalg.norm(v_n)))
        axiszr =np.array([0.0, 0.0, -1.0])
        n_x = v/np.linalg.norm(v)

        # print("nx = ",n_x)

        # R_z = rotation_matrix_z(angle)
        R_z = np.hstack([n_x.reshape(3,-1),
                         np.cross(axiszr, n_x).reshape(3,-1),
                         axiszr.reshape(3,-1)])
        R_z = R_z.T

        rf = R_z @ fr.reshape(3,-1)
        rm = R_z @ mr.reshape(3,-1)
        print("pz = ", rm[0])


        # if (rf[1] > 1e-5):
        #     pz = rm[0]/rf[1]
        # else:
        #     pz = 0.0
        pz = -0.05

        # R_z = np.array([[np.cos(angle), -np.sin(angle), 0.0],
        #         [np.sin(angle), np.cos(angle), 0.0],
        #         [0.0,             0.0,             1.0]])
        
        
        # if((R_z @ f_lin.reshape(3,-1))[1] > 0.001):
        #     print("angle = ", np.dot(v, v_n))
        #     print("f_lin = ", f_lin)
        #     print("R_z = ", R_z)
            # print("projection !!!!!!!!!!!!!!!!!!!!!!!!!! {0}".format((R_z @ f_lin.reshape(3,-1))[1]))

        RT[:3,:3] = R_z
        RT[2,3] = pz
        # print("pz = ",pz)
        temp = R_z@ fr.reshape(3,-1)
        # print("R_z@  fr x= ",temp[0])
        # print("R_z@  fr y= ",temp[1])
        # if(temp[1]>1e-3):
        #     print("np.dot(v, v_n) = ",np.dot(v, v_n))
        #     print("np.linalg.norm(v) = ",np.linalg.norm(v))
        # print("R_z@  fr z= ",temp[2])
        # print("RT = ",RT)
    # else:
    #     print("projection !!!!!!!!!!!!!!!!!!!!!")
    

    return RT

def find_tile_angle(f,mf,alpha_last):
    fx = f[0]
    fz = f[2]
    my = mf[1]

    if(fz < 0.0 or my <0):
        return alpha_last
    
    mu = 0.1
    r = 0.005
    temp = (
        mu*mu*(r*fz-my)-
        r*fx*mu+
        my
        )

    numr = np.sqrt(
        r*r*(fx-mu*fz)*(fx-mu*fz) +
        r*(fz - mu*fx)*(
        mu*mu*(r*fz-my)-
        r*fx*mu+
        my)
        
    )- r*(
        fx-mu*fz
    )

    denr = r * (fz - mu * fx)
    alpha = 2 * np.arctan2(numr,denr)
    print("pre alpha = ",alpha)


    

    # if(np.isnan(numr) and my>0):
    #     print("my = ", my)
    #     print("(fz - mu*fx) = ", (fz - mu*fx))


    # if(my<0):
    #     print("mu*mu*(r*fz-my)-r*fx*mu+my = ",(-r*fx*mu+my))
    #     print("temp = ",temp)
    #     print("my = ",my)


    if (alpha > math.pi):
        alpha = alpha - 2*math.pi
    elif (alpha < -math.pi):
        alpha = 2*math.pi +alpha

    # print(alpha)
    
    


    return alpha
        



class Planner:

    def __init__(self):
        # ee_link = 'storz_endoscope_link_cm_optical'
        ee_link = 'storz_tilt_endoscope_link_cm_optical'
        T = 20
        robot = optas.RobotModel(urdf_filename='resources/lbr_with_tilt_endoscope.urdf', time_derivs=[0, 1])
        name = robot.get_name()
        robot.add_base_frame('pybullet_world', [1, 0, 0])
        builder = optas.OptimizationBuilder(T, robots=robot)
        initial_configuration = builder.add_parameter('initial_configuration', robot.ndof)
        goal_position = builder.add_parameter('goal_position', 3)
        goal_axis_align = builder.add_parameter('goal_axis_align', 3)
        obstacle = builder.add_parameter('obstacle', 3)
        duration = builder.add_parameter('duration')
        dt = duration/float(T-1)
        builder.integrate_model_states(name, 1, dt)
        builder.initial_configuration(name, initial_configuration)
        builder.initial_configuration(name, time_deriv=1)
        builder.enforce_model_limits(name)
        qF = builder.get_model_state(name, -1)
        TF = robot.get_global_link_transform(ee_link, qF)
        pF = TF[:3, 3]
        zF = TF[:3, 2]
        builder.add_equality_constraint('final_position', pF, goal_position)
        builder.add_equality_constraint('final_axis_align', zF, goal_axis_align)
        dQ = builder.get_model_states(name, time_deriv=1)
        builder.add_cost_term('minimize_joint_velocity', 20.*optas.sumsqr(dQ))
        # manipF = robot.get_global_manipulability(ee_link, qF)
        # builder.add_cost_term('max_manipulability', -100.*manipF**2)
        self.solver = optas.CasADiSolver(builder.build()).setup('ipopt')
        self.name = name
        self.endoscope_tip_position = robot.get_global_link_position_function(ee_link)
        self.T = T
        self.robot = robot

    def __call__(self, q0, p, z, dur, o):
        self.solver.reset_parameters({
            'initial_configuration': q0,
            'goal_position': p,
            'goal_axis_align': z,
            'duration': dur,
            'obstacle': o,
        })
        solution = self.solver.solve()
        plan = self.solver.interpolate(solution[f'{self.name}/q'], dur, fill_value='extrapolate')
        return plan

planner = Planner()

##########################################################
## Setup simulator

if 'gui' in sys.argv:
    connect_args = [pb.GUI]
else:
    connect_args = [pb.DIRECT]

pb.connect(
    *connect_args,
    # options='--mp4=videos/plan_peg_in_hole_unfixed_hy3_2.mp4',
)
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
    cameraYaw=0,
    cameraPitch=-80,
    cameraTargetPosition=box_base_position,
)
pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)

##########################################################
## Load environment

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
    springElasticStiffness=500,
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
    springElasticStiffness=500,
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

hole_initial_central_position = box_base_position.copy()

cloth_left_base_position = hole_initial_central_position + np.array([-0.03, 0, 0])
cloth_right_base_position = hole_initial_central_position + np.array([0.03, 0, 0])
cloth_rotation = pb.getQuaternionFromEuler(np.deg2rad([45, 0, 0]))

pb.resetBasePositionAndOrientation(cloth_left_id, cloth_left_base_position , cloth_rotation)
pb.resetBasePositionAndOrientation(cloth_right_id,  cloth_right_base_position, cloth_rotation)
pb.resetBasePositionAndOrientation(box_id, hole_initial_central_position, cloth_rotation)

##########################################################
## Setup controller

insertion_depth = 0.03

class Controller:

    def __init__(self, dt):
        self.response_gain = 1.0
        ee_link = 'storz_tilt_endoscope_link_cm_optical'
        T = 2
        robot = optas.RobotModel(urdf_filename='resources/lbr_with_tilt_endoscope.urdf', time_derivs=[0, 1])
        name = robot.get_name()
        robot.add_base_frame('pybullet_world', [1, 0, 0])
        builder = optas.OptimizationBuilder(T, robots=robot)

        qc = builder.add_parameter('qc', robot.ndof)
        box_pos = builder.add_parameter('box_pos', 3)
        axis_align = builder.add_parameter('axis_align', 3)
        response = builder.add_parameter('response', 3)
        response_r = builder.add_parameter('response_r', 3)

        eff_goal = box_pos + insertion_depth*axis_align


        q0 = builder.get_model_state(name, 0)
        dq = builder.get_model_state(name, t=0, time_deriv=1)
        qF = builder.get_model_state(name, 1)
        builder.add_equality_constraint('initial', q0, qc)
        builder.add_equality_constraint('dynamics', qc + dt*dq, qF)
        # qF = qc + dt*dq
        # Get jacobian
        J = robot.get_global_link_geometric_jacobian(ee_link, qc)
        # Get end-effector velocity
        dp = J @ dq
        
        builder.enforce_model_limits(name)

        # pc = robot.get_global_link_position(ee_link, qc)
        
        
        
        TfC = robot.get_global_link_transform(ee_link, qc)
        pc = TfC[:3, 3]
        Rc = TfC[:3, :3]
        Om = skew(dp[3:]) 

        # Turn the reference frame to Cartesian space of EE (ee_link)
        p = Rc.T @ dt @ dp[:3]
        R = Rc.T @ (Om * dt + I3())


        pF = dt @ dp[:3] + pc
        # pF = pc
        print("dp  = {0}".format(dp.size()))
        
        # print("eff_goal size = {0}".format(eff_goal.size()))
        RF = (Om * dt + I3()) @ Rc  
        zF = RF[:3,2]

        p_box_pos = Rc.T @ (box_pos - pc)

        # Minimize distance from end-effector position to trocar axis
        # Hybrid force-position decomposition (The defined projection vector is the subspace for force)
        # nf = optas.DM([0.0, 0.0, 1.0])
        # compliance_virtual_force
        Bz = 0.004
        Bx = 0.00001
        By = 0.00001
        fz = 1.0
        alpha = -2000.0
        beta = -1000.0
        # delta = 0.08
        fRot = 0.03
        cvf = Rc.T  @ response
        
        diffp = optas.diag([1.0, 1.0, 0.0]) @ (p- p_box_pos + optas.diag([Bx, By, 0.0]) @ cvf )

        # Minimize distance between end-effector position and goal
        # diffzz = p_box_pos[2]

        diffpp = optas.sumsqr(optas.diag([1.0, 1.0, 0.0]) @ p_box_pos)
        ff = fz * optas.np.exp(alpha * diffpp)
        diffFl = optas.diag([0.0, 0.0, 1.0]) @ (optas.diag([0.0, 0.0, Bz]) @(cvf + ff * optas.DM([0.0, 0.0, 1.0])) - Rc.T @dp[:3])



        W_p = optas.diag([1e3, 1e3, 1e3])
        builder.add_cost_term('dist_to_axis_align', diffp.T @ W_p @ diffp)
        W_f = optas.diag([1e1, 1e1, 1e1])
        builder.add_cost_term("match_f", diffFl.T @ W_f @ diffFl)

        # Align end-effector with trocar axis
        # builder.add_cost_term('eff_axis_align', 50.*optas.sumsqr(zF - axis_align))

        fRp = fRot * optas.np.exp(beta * optas.sumsqr(p_box_pos[2]))
        rvf = Rc.T @ optas.diag([10.0, 10.0, 10.0])@response_r
        # diffFR =   Rc.T @dp[3:] + optas.diag([5.0, 5.0, 0.0])@ rvf
        diffFR =   optas.diag([5.0, 5.0, 0.0])@ rvf
        
        # deltaF = skew(response_r) 
        RFORF = (skew(diffFR *0.01)+ I3()) @ Rc 
        # print("deltaF size = {0}".format(deltaF.size()))

        #此处有一些问题
        axis_align_update = RFORF[:3,2]
        axis_align_update = axis_align

        # W_fr = optas.diag([1e0, 1e0, 1e0])
        builder.add_cost_term("eff_axis_align", 500.*optas.sumsqr(zF - axis_align_update))
        


        # Minimize joint velocity
        W = 0.2*optas.diag([2, 2, 2, 1, 1, 1, 1])
        builder.add_cost_term('min_dq', dq.T@W@dq)

        


        builder.add_leq_inequality_constraint(
            "dq1", dq[0] * dq[0], 1e-4
        )
        builder.add_leq_inequality_constraint(
            "dq2", dq[1] * dq[1], 1e-4
        )
        builder.add_leq_inequality_constraint(
            "dq3", dq[2] * dq[2], 1e-4
        )
        builder.add_leq_inequality_constraint(
            "dq4", dq[3] * dq[3], 1e-4
        )
        builder.add_leq_inequality_constraint(
            "dq5", dq[4] * dq[4], 1e-4
        )
        builder.add_leq_inequality_constraint(
            "dq6", dq[5] * dq[5], 1e-4
        )
        builder.add_leq_inequality_constraint(
            "dq7", dq[6] * dq[6], 1e-4
        )
        


        self.solver = optas.ScipyMinimizeSolver(builder.build()).setup('SLSQP')
        self.name = name
        self.solution = None

        c = optas.sumsqr(pc - eff_goal)
        
        self._finish_criteria = optas.Function('finish', [qc, box_pos, axis_align], [c])
        self._OrientationAlgin = optas.Function('OrientationAlgin', [qc, response_r], [axis_align_update])
        # self._fRp = optas.Function('para', [qF, box_pos, axis_align], [p_box_pos])

    def finish_criteria(self, q, bp, z):
        tol = 0.005**2
        f = self._finish_criteria(q, bp, z)
        return f < tol
    
    def OrientationAlgin(self, q, response_r):
        z_axis = self._OrientationAlgin(q, response_r)
        return z_axis.toarray().flatten()

    def __call__(self, qc, bp, z, f, f_r):

        resp = self.response_gain*f
        resp_r = self.response_gain*f_r
        # print(resp)
        if self.solution is not None:
            self.solver.reset_initial_seed(self.solution)
        else:
            self.solver.reset_initial_seed({f'{self.name}/q': optas.horzcat(qc, qc)})
        self.solver.reset_parameters({'qc': qc, 'box_pos': bp, 'axis_align': z, 'response': resp, 'response_r': resp_r})
        t0 = perf_counter()
        self.solution = self.solver.solve()
        t1 = perf_counter()
        dur = t1 - t0
        self.dur = dur
        # print("self._fRp = {0}".format(self._fRp(qc, bp, z)))
        # print(f"-------\nSolver duration (s, hz): {dur}, {1/dur}")
        return self.solution[f'{self.name}/q'][:, -1].toarray().flatten()

    def get_solver_duration(self):
        return self.dur

controller = Controller(time_step)

##########################################################
## Load visualization objects

# goal_sphere_vis_id = pb.createVisualShape(
#     pb.GEOM_SPHERE,
#     radius=0.005,
#     rgbaColor=[0, 1, 0, 0.25]
# )
# goal_sphere_id = pb.createMultiBody(baseVisualShapeIndex=goal_sphere_vis_id)

# obstacle_sphere_vis_id = pb.createVisualShape(
#     pb.GEOM_SPHERE,
#     radius=obstacle_radius,
#     rgbaColor=[1, 0, 0, 0.5]
# )
# obstacle_sphere_id = pb.createMultiBody(baseVisualShapeIndex=obstacle_sphere_vis_id)

# endoscope_tip_sphere_vis_id = pb.createVisualShape(
#     pb.GEOM_SPHERE,
#     radius=endoscope_tip_radii,
#     rgbaColor=[0, 0, 1, 0.5]
# )
# endoscope_tip_sphere_id = pb.createMultiBody(baseVisualShapeIndex=endoscope_tip_sphere_vis_id)

##########################################################
## Load robot

class Robot:

    def __init__(self):
        self.id = pb.loadURDF(
            'lbr_with_tilt_endoscope.urdf',
            basePosition=[1, 0, 0],
        )
        self._dq = np.zeros(7)
        self._robot = optas.RobotModel(urdf_filename='resources/lbr_with_tilt_endoscope.urdf', time_derivs=[0, 1])
        self._robot.add_base_frame('pybullet_world', [1, 0, 0])
        # self._J = self._robot.get_global_geometric_jacobian_function('lbr_link_ee')
        self._J = self._robot.get_global_link_geometric_jacobian_function('storz_tilt_endoscope_link_cm_optical')
        self._p = self._robot.get_global_link_position_function('storz_tilt_endoscope_link_cm_optical')
        self._Tf = self._robot.get_global_link_transform_function('storz_tilt_endoscope_link_cm_optical')

    def Tf(self):
        return self._Tf(self.q()).toarray()

    def p(self):
        return self._p(self.q()).toarray().flatten()

    def J(self, q):
        return self._J(q).toarray()

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

    def update(self):
        self._dq = self.dq()

    def q(self):
        return np.array([state[0] for state in pb.getJointStates(self.id, self.revolute_joint_indices)])

    def dq(self):
        return np.array([state[1] for state in pb.getJointStates(self.id, self.revolute_joint_indices)])

    def ddq(self):
        return (self.dq() - self._dq)/time_step

    def tau(self):
        return np.array([state[3] for state in pb.getJointStates(self.id, self.revolute_joint_indices)])

    def tau_ext(self):
        tau = self.tau()
        q = self.q().tolist()
        dq = self.dq().tolist()
        ddq = self.ddq().tolist()
        tau_ext = np.array(pb.calculateInverseDynamics(self.id, q, dq, ddq)) - tau
        # lim = 1.1*np.array([-7.898,  3.47 , -1.781, -5.007,  1.453,  1.878,  2.562])
        lim = 2*np.ones(7)

        for i in range(7):
            if -lim[i] <= tau_ext[i] < lim[i]:
                tau_ext[i] = 0.

        # print("=================================")
        # print(f"{tau_ext=}")
        # print("=================================")
        return tau_ext

    def f_ext(self):
        J = self.J(self.q())
        tau_ext = self.tau_ext()
        J_inv = np.linalg.pinv(J, rcond=0.05)
        f_ext = J_inv.T @ tau_ext
        f_ext *= np.array([1.0, 1.0, 1.0, 1, 1, 1])

        # print("=================================")
        # print("== Compute f_ext")

        # print(f"raw: {f_ext=}")

        # print("=================================")




        # f_threshold = np.array([6.0, 6.0, 6.0, 1.0, 1.0, 1.0])

        # f_ext = np.where(
        #     abs(f_ext) > f_threshold,
        #     np.sign(f_ext) * (abs(f_ext) - f_threshold),
        #     0.0,
        # )
        return f_ext

    def reset(self, q, deg=False):
        for joint_index, joint_position in zip(self.revolute_joint_indices, q):
            pb.resetJointState(self.id, joint_index, joint_position)

    def cmd(self, q):
        pb.setJointMotorControlArray(
            self.id,
            self.revolute_joint_indices,
            pb.POSITION_CONTROL,
            q,
        )

    def cmd_vel(self, dq):
        pb.setJointMotorControlArray(
            self.id,
            self.revolute_joint_indices,
            pb.VELOCITY_CONTROL,
            dq,
        )

robot = Robot()

q0 = np.deg2rad([-40, -45, 0, 90, 0, -45, 0])
robot.reset(q0)
robot.cmd(q0)

##########################################################
## Wait for box to come to almost rest

t = 0.
rest_wait_time = 5.  # secs
time_factor_for_rest = 0.0001

while pb.isConnected():
    pb.stepSimulation()
    sleep(time_step*time_factor_for_rest)
    t += time_step
    robot.update()
    if t > rest_wait_time:
        break

##########################################################
## Plan trajectory from initial configuration to pre-insertion

# np.random.seed(10)

def get_box_pose(noise=0.01):
    p, r = pb.getBasePositionAndOrientation(box_id)
    noisep = np.random.uniform(-noise, noise, size=(len(p),))
    noiser = np.random.uniform(-noise, noise, size=(len(r),))
    noiser_offset = np.array([0.5,0.5,0.5,0.0])
    return np.array(p) + noisep, np.array(r) + noiser+noiser_offset

def get_box_pose_true(noise=0.01):
    p, r = pb.getBasePositionAndOrientation(box_id)
    return np.array(p) , np.array(r) 

box_base_position, box_base_orientation = get_box_pose()

R_box = Rot.from_quat(box_base_orientation).as_matrix()

pre_insertion_dur = 4.
pre_insertion_offset = R_box[:3, 1] * 0.02
goal_p = box_base_position + pre_insertion_offset
goal_z = -R_box[:3, 1]
pre_insertion_plan = planner(robot.q(), goal_p, goal_z, pre_insertion_dur, [0, 0, 0])

##########################################################
## Execute pre-insertion plan

def exec_plan(plan, dur, time_factor=1.):

    t = 0.

    while pb.isConnected():
        # print(robot.tau_ext())
        robot.cmd(plan(t))
        pb.stepSimulation()
        sleep(time_step*time_factor)
        t += time_step

        robot.update()

        if t > dur:
            t = 0
            break

    # TODO: investigate why this is required
    for _ in range(sampling_freq):
        pb.stepSimulation()
        sleep(time_step*time_factor)
        robot.update()

exec_plan(pre_insertion_plan, pre_insertion_dur, time_factor=0.00001)

# quit()

##########################################################
## Run insertion using feedback controller

class VisualizeLine:

    def __init__(self, use=True):
        self.id = None
        self.use = use

    def __call__(self, from_pos, to_pos):
        if not self.use: return
        if self.id is not None:
            self.id = pb.addUserDebugLine(lineFromXYZ=from_pos.tolist(), lineToXYZ=to_pos.tolist(), replaceItemUniqueId=self.id)
        else:
            self.id = pb.addUserDebugLine(lineFromXYZ=from_pos.tolist(), lineToXYZ=to_pos.tolist())

visualize_line = VisualizeLine(True)
visualize_line2 = VisualizeLine(True)

def is_finished(q, bp, z):
    return controller.finish_criteria(q, bp, z)

t = 0.
t_data = []
f_ext_data = []

f_ext_window = []
max_window_len = 50

time_out = 130.
max_f_ext_lin_nrm = 100000.0

success = True

time_factor = 0.000001

solver_durations = []

q = pre_insertion_plan(pre_insertion_dur)

q_data = []
box_base_position_data = []
box_base_orientation_data = []
force_record_x = []
force_record_y = []
force_record_z = []

force_record_x_a = []
force_record_y_a = []
force_record_z_a = []


force_record_mx = []
force_record_my = []
force_record_mz = []
ts =[]

order = 4
cutoff_freq = 0.2
bk, ak = signal.butter(order, cutoff_freq, btype='low', analog=False, output='ba')

# 应用滤波器
alpha1 = 0.03
alpha2 = 0.03

beta1 = 0.1
f_ext_lin = np.array([0.0]*3)
f_ext_rot = np.array([0.0]*3)

xs = []
ys = []
zs = []
rxs = []
rys = []
rzs = []
rxs1 = []
rys1 = []
rzs1 = []
alphas = []
angles = []
alpha = 0.0

x_rs = []
y_rs = []
z_rs = []

box_base_position_last = np.array([0.0]*3)

while pb.isConnected():

    # print(robot.tau_ext())
    t_data.append(t)
    q_data.append(np.asarray(q).flatten().tolist())
    f_ext = robot.f_ext()
    # print("f_ext = {0}".format(f_ext.shape))
    f_ext_lin_nrm = np.linalg.norm(f_ext[:3])
    f_ext_data.append(f_ext.copy().tolist())
    f_ext_window.append(f_ext)
    if len(f_ext_window) > max_window_len:
        f_ext_window.pop(0)
    # f_ext_nrm = f_ext[:3] / np.linalg.norm(f_ext[:3])
    # print(f_ext_nrm)
    peff = robot.p()
    # visualize_line(peff, peff + 0.4*f_ext_nrm)

    # Sensing interaction (generate end-effector motion)
    f_ext_lin = alpha1*f_ext[:3] + (1-alpha1)*f_ext_lin
    f_ext_rot = alpha2*f_ext[3:] + (1- alpha2)*f_ext_rot
    # print("f_ext_rot = {0}".format(f_ext_rot))

    # print("-------------")
    # print(f"{f_ext_lin=}")
    # print("-------------")

    # f_ext_smooth = np.mean(f_ext_window, axis=0)
    # f_ext_smooth_nrm = f_ext_smooth[:3] / np.linalg.norm(f_ext_smooth[:3])

    # f_ext_smooth = np.mean(f_ext_window, axis=0)
    # f_ext_smooth_nrm = f_ext_smooth[3:] / np.linalg.norm(f_ext_smooth[3:])
    # visualize_line(peff, peff + 0.4*f_ext_smooth_nrm)
    # visualize_line(peff, f_ext_rot)

    # Sensing of box
    box_base_position, box_base_orientation = get_box_pose()
    bp, bo = get_box_pose_true()

    box_base_position = beta1*box_base_position + (1-beta1) * box_base_position_last
    box_base_position_last = box_base_position




    R_box = Rot.from_quat(box_base_orientation).as_matrix()
    goal_z = -R_box[:3, 1]

    R_box_b = Rot.from_quat(bo).as_matrix()
    goal_z_b = -R_box_b[:3, 1]



    box_base_position_data.append(np.asarray(box_base_position).tolist())
    box_base_orientation_data.append(np.asarray(box_base_orientation).tolist())

    # Call controller for next joint state and update robot
    # zaxis_up=controller.OrientationAlgin(q, f_ext_rot)
    # visualize_line(peff, peff+0.4*zaxis_up)
    Tf = robot.Tf()
    Rf = Tf[:3,:3]
    tf = Tf[:3, 3]
    actual_z = Tf[:3, 2]


    #related motion
    R_rt = Rf.T @ R_box
    t_rt = -Rf.T @ tf.reshape(3,1) + Rf.T @ box_base_position.reshape(3,1)
    # r_eul = Rot.from_matrix(R_rt).as_euler()



    RT = find_xz_plane_RT(f_ext_lin, f_ext_rot,Tf)
    f_ext_lin_temp = copy.deepcopy(RT[:3,:3]@ Rf.T @f_ext_lin.reshape(3,-1))

    # print("!!! f_ext_lin_temp[1] = ",f_ext_lin_temp[1])

    R_tw = RT[:3,:3]@ Rf.T
    R_wt = R_tw.T

    f_ext_rot_temp = copy.deepcopy(RT[:3,:3]@ Rf.T @f_ext_rot.reshape(3,-1) + skew_np(RT[:3,3].flatten()) @ RT[:3,:3]@ Rf.T @f_ext_lin.reshape(3,-1))
    # print("f_ext_rot_temp = ",skew_np(RT[:3,3].flatten()) @ RT[:3,:3]@ f_ext_lin.reshape(3,-1))
    # print("f_ext_lin_temp = ",f_ext_lin_temp[2])

    if(f_ext_lin_temp[2]> 0.5):
        alpha = find_tile_angle(f_ext_lin_temp,f_ext_rot_temp,alpha)
        # print("aa alpha1 = ",alpha)
        if(np.isnan(alpha)):
            alpha =0.0
        # print("aa alpha2 = ",alpha)
        alpha = float(alpha)
        # print("aa alpha3 = ",alpha)
    
    goal_z_new = RT[:3,:3]@calculate_new_axis(0.1*alpha, goal_z)
    q = controller(q, box_base_position, goal_z_b, f_ext_lin, f_ext_rot)

    # print(alpha)

    robot.cmd(q)
    solver_durations.append(controller.get_solver_duration())
    # print("goal_z_new",goal_z_new.flatten())
    # print("box_base_position",box_base_position)

    # visualize_line(box_base_position, box_base_position+goal_z_new.flatten()*0.1)
    # visualize_line(peff, goal_z)
    # Update simulation
    pb.stepSimulation()
    sleep(time_step*time_factor)
    # sleep(100*time_step)

    robot.update()
    transl = robot.p()
    qdv = robot.dq()
    Jr = robot.J(robot.q())
    vr = Jr @ qdv 
    # print("transl = {0}".format(transl))
    # print("box_base_position = {0}".format(box_base_position))

    
    # visualize_line(Tf[:3,3].flatten(), Tf[:3,3].flatten()+f_ext_lin.flatten()*0.1)
    # visualize_line(box_base_position, box_base_position+np.array([0.0,0.0,1.0])*0.1)
    # print("Tf = {0}".format(Tf))
    # transl = robot.robot.get_global_link_position("end_effector_ball", q)
    # Jl = robot.robot.get_global_link_linear_jacobian("end_effector_ball", q)
    # Rc = robot.robot.get_global_link_rotation("end_effector_ball", q)

    
    force_record_x.append(f_ext_lin[0])
    force_record_y.append(f_ext_lin[1])
    force_record_z.append(f_ext_lin[2])
    





    R_boxb = Rot.from_quat(bo).as_matrix()
    goal_zb = -R_boxb[:3, 1]

    angle = np.dot(goal_zb, goal_z)/(np.linalg.norm(goal_z)* np.linalg.norm(goal_zb))

    alphas.append(alpha)
    angles.append(angle)

    # if(float(f_ext_lin_temp[1]) > 0.0001):
    #     print(RT)
    #     print(float(f_ext_lin_temp[1]))

    force_record_x_a.append(f_ext_lin_temp[0])
    force_record_y_a.append(f_ext_lin_temp[1])
    # print("f_ext_lin_temp[1] = ",f_ext_lin_temp[1])
    force_record_z_a.append(f_ext_lin_temp[2])

    force_record_mx.append(f_ext_rot_temp[0])
    force_record_my.append(f_ext_rot_temp[1])
    force_record_mz.append(f_ext_rot_temp[2])

    xs.append(box_base_position[0] - transl[0])
    ys.append(box_base_position[1] - transl[1])
    zs.append(box_base_position[2] - transl[2])

    rxs.append(goal_z[0])
    rys.append(goal_z[1])
    rzs.append(goal_z[2])

    rxs1.append(actual_z[0])
    rys1.append(actual_z[1])
    rzs1.append(actual_z[2])

    x_rs.append(t_rt[0])
    y_rs.append(t_rt[1])
    z_rs.append(t_rt[2])

    # 计算旋转轴
    axis1 = np.cross(goal_z, goal_zb)  # 旋转轴为两个单位向量的叉乘
    axis1 /= np.linalg.norm(axis1)  # 将旋转轴单位化
    # print("axis1 = ",axis1)


    # 计算旋转轴2
    axis_x = R_wt[:,0]
    axis_z = R_wt[:,2]
    axis2 = np.cross(axis_z, axis_x)  # 旋转轴为两个单位向量的叉乘
    axis2 /= np.linalg.norm(axis2)  # 将旋转轴单位化
    print("axis1 = {0} axis2 = {1}".format(axis1, axis2))
    # visualize_line(Tf[:3,3].flatten(), Tf[:3,3].flatten()+axis1.flatten()*0.1)
    # visualize_line(Tf[:3,3].flatten(), Tf[:3,3].flatten()-axis2.flatten()*0.1)
    # visualize_line(Tf[:3,3].flatten(), Tf[:3,3].flatten()+f_ext_rot.flatten()*0.1)
    visualize_line2(Tf[:3,3].flatten(), Tf[:3,3].flatten()+vr[:3].flatten()*100.0)
    # visualize_line2(Tf[:3,3].flatten(), Tf[:3,3].flatten()+axis2.flatten()*0.1)

    ts.append(t)



    t += time_step

    if f_ext_lin_nrm >= max_f_ext_lin_nrm:
        success = False
        print("Too much force", f_ext_lin_nrm)
        break

    if is_finished(q, box_base_position, goal_z):
        print("Finished")
        break

    if t > time_out:
        print("Session timed out")
        success = False
        break

pb.disconnect()

n_t = len(t_data)

D = np.concatenate((
    np.array(t_data).reshape(n_t, 1),  # 1
    np.array(f_ext_data),  # 6
    np.array(q_data),  # 7
    np.array(box_base_position_data), # 3
    np.array(box_base_orientation_data),  # 4
), axis=1)

print("total_time = {0}".format(t))
# D = np.zeros((len(t_data), 1 + 6 + 7 + 7))
# D[:,0] = t_data
# D[:,1:5] = np.array(f_ext_data)
# D[:,6:] =

success_msg = 'success' if success else 'failed'
stamp = time_ns()

filename = f'f_ext_v4_{success_msg}_{stamp}.csv'

if 'save' in sys.argv:
    np.savetxt(filename, D, delimiter=',')
    print("Saved data")

##########################################################
## Keep simulator running

# while pb.isConnected():
#     pb.stepSimulation()
#     sleep(time_step)

##########################################################
## Plot solver durations

# fig, ax = plt.subplots(layout='constrained')
# ax.plot(t_data, 1000*np.array(solver_durations), '_')
# plt.show()

print("Success:", success)
plt.subplot(331)
plt.plot(ts, force_record_x, label ='force x')
plt.plot(ts, force_record_x_a, label ='pos err x')
plt.title("Force Measurment along x-axis")
plt.subplot(332)
plt.plot(ts, force_record_y, label ='force y')
plt.plot(ts, force_record_y_a, label ='pos err y')
plt.title("Force Measurment along y-axis")
plt.subplot(333)
plt.plot(ts, force_record_z, label ='force z')
plt.plot(ts, force_record_z_a, label ='pos err z')
plt.title("Force Measurment along z-axis")
plt.subplot(334)
plt.plot(ts, force_record_mx, label ='Actual x')
# plt.plot(ts, rxs, label ='pos err z')
# plt.plot(ts, rxs1, label ='pos err z')
plt.title("Torque Measurment around x-axis")
plt.subplot(335)
plt.plot(ts, force_record_my, label ='Actual y')
# plt.plot(ts, rys, label ='pos err z')
# plt.plot(ts, rys1, label ='pos err z')
plt.title("Torque Measurment around y-axis")
plt.subplot(336)
plt.plot(ts, force_record_mz, label ='Actual z')
# plt.plot(ts, rzs, label ='pos err z')
# plt.plot(ts, rzs1, label ='pos err z')
plt.title("Torque Measurment around z-axis")
plt.subplot(337)
# plt.plot(ts, )
plt.plot(ts, x_rs)
plt.subplot(338)
plt.plot(ts, y_rs)
plt.subplot(339)
plt.plot(ts, z_rs)
# plt.plot(ts,alphas)
# plt.plot(ts,angles)
plt.show()

