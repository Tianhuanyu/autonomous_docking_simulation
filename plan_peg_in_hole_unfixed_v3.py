import optas
import sys
import numpy as np
import pybullet as pb
import matplotlib.pyplot as plt
from time import sleep, time, perf_counter, time_ns
from scipy.spatial.transform import Rotation as Rot

np.set_printoptions(precision=3, linewidth=1000, suppress=True)

##########################################################
## Setup planner

obstacle_radius = 0.01
endoscope_tip_radii = 0.0025

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
        manipF = robot.get_global_manipulability(ee_link, qF)
        builder.add_cost_term('max_manipulability', -100.*manipF**2)
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
    # options='--mp4=videos/plan_peg_in_hole_unfixed_v4.mp4',
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
    cameraYaw=-40,
    cameraPitch=30,
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
        self.response_gain = 0.4
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
        eff_goal = box_pos + insertion_depth*axis_align
        q0 = builder.get_model_state(name, 0)
        dq = builder.get_model_state(name, 0, time_deriv=1)
        qF = builder.get_model_state(name, 1)
        builder.add_equality_constraint('initial', q0, qc)
        builder.add_equality_constraint('dynamics', q0 + dt*dq, qF)
        builder.enforce_model_limits(name)
        TfF = robot.get_global_link_transform(ee_link, qF)
        pF = TfF[:3, 3]
        zF = TfF[:3, 2]
        insertion_point = pF - zF*insertion_depth

        alpha = optas.dot(pF - box_pos, axis_align)
        p_close2eff = box_pos + alpha*axis_align
        dist_sqr = optas.sumsqr(p_close2eff - pF)
        builder.add_cost_term('min_dist_close2eff', 100*dist_sqr)

        builder.add_cost_term('insertion_point', 900.*optas.sumsqr(insertion_point - box_pos))
        builder.add_cost_term('eff_goal', 900*optas.sumsqr(pF - eff_goal))

        builder.add_cost_term('eff_axis_align', 5000*optas.sumsqr(zF - axis_align))
        builder.add_cost_term('min_joint_vel', 5*optas.sumsqr(dq))

        # ------------------------------------------------------------------------------
        # Note: using this produces worse behaviour than what im using below...strange!
        # J = robot.get_global_geometric_jacobian('lbr_link_ee', qc)
        # deff = J @ dq
        effc = robot.get_global_link_position('lbr_link_ee', qc)
        effF = robot.get_global_link_position('lbr_link_ee', qF)
        deff = (effF - effc)/dt
        builder.add_cost_term('min_eff_vel', optas.sumsqr(deff))
        # ------------------------------------------------------------------------------

        builder.add_cost_term('respond_to_interaction', 15*optas.sumsqr(response - deff))

        # self.solver = optas.CasADiSolver(builder.build()).setup('ipopt')
        # self.solver = optas.CasADiSolver(builder.build()).setup('sqpmethod')
        self.solver = optas.ScipyMinimizeSolver(builder.build()).setup('SLSQP')
        self.name = name
        self.solution = None

        c = optas.sumsqr(pF - eff_goal)
        self._finish_criteria = optas.Function('finish', [qF, box_pos, axis_align], [c])

    def finish_criteria(self, q, bp, z):
        tol = 0.005**2
        f = self._finish_criteria(q, bp, z)
        return f < tol

    def __call__(self, qc, bp, z, f):
        resp = self.response_gain*f
        if self.solution is not None:
            self.solver.reset_initial_seed(self.solution)
        else:
            self.solver.reset_initial_seed({f'{self.name}/q': optas.horzcat(qc, qc)})
        self.solver.reset_parameters({'qc': qc, 'box_pos': bp, 'axis_align': z, 'response': resp})
        t0 = perf_counter()
        self.solution = self.solver.solve()
        t1 = perf_counter()
        dur = t1 - t0
        self.dur = dur
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
        self._J = self._robot.get_global_geometric_jacobian_function('storz_tilt_endoscope_link_cm_optical')
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
        f_ext *= np.array([0.01, 0.01, 0.01, 1, 1, 1])

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

def get_box_pose(noise=0.005):
    p, r = pb.getBasePositionAndOrientation(box_id)
    noisep = np.random.uniform(-noise, noise, size=(len(p),))
    noiser = np.random.uniform(-noise, noise, size=(len(r),))
    return np.array(p) + noisep, np.array(r) + noiser

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
        print(robot.tau_ext())
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

visualize_line = VisualizeLine(False)

def is_finished(q, bp, z):
    return controller.finish_criteria(q, bp, z)

t = 0.
t_data = []
f_ext_data = []

f_ext_window = []
max_window_len = 50

time_out = 30.
max_f_ext_lin_nrm = 0.6

success = True

time_factor = 0.000001

solver_durations = []

q = pre_insertion_plan(pre_insertion_dur)

while pb.isConnected():

    # print(robot.tau_ext())
    t_data.append(t)
    f_ext = robot.f_ext()
    f_ext_lin_nrm = np.linalg.norm(f_ext[:3])
    f_ext_data.append(f_ext.copy().tolist())
    f_ext_window.append(f_ext)
    if len(f_ext_window) > max_window_len:
        f_ext_window.pop(0)
    f_ext_nrm = f_ext[:3] / np.linalg.norm(f_ext[:3])
    # print(f_ext_nrm)
    peff = robot.p()
    # visualize_line(peff, peff + 0.4*f_ext_nrm)

    # Sensing interaction (generate end-effector motion)
    f_ext_lin = f_ext[:3]

    # print("-------------")
    # print(f"{f_ext_lin=}")
    # print("-------------")

    f_ext_smooth = np.mean(f_ext_window, axis=0)
    f_ext_smooth_nrm = f_ext_smooth[:3] / np.linalg.norm(f_ext_smooth[:3])
    # visualize_line(peff, peff + 0.4*f_ext_smooth_nrm)
    visualize_line(peff, peff + f_ext_lin)

    # Sensing of box
    box_base_position, box_base_orientation = get_box_pose()
    R_box = Rot.from_quat(box_base_orientation).as_matrix()
    goal_z = -R_box[:3, 1]

    # Call controller for next joint state and update robot
    q = controller(q, box_base_position, goal_z, f_ext_lin)
    robot.cmd(q)
    solver_durations.append(controller.get_solver_duration())

    # Update simulation
    pb.stepSimulation()
    sleep(time_step*time_factor)
    # sleep(100*time_step)

    robot.update()

    t += time_step

    if f_ext_lin_nrm >= max_f_ext_lin_nrm:
        success = False
        break

    if is_finished(q, box_base_position, goal_z):
        print("Finished")
        break

    if t > time_out:
        success = False
        break

pb.disconnect()

D = np.zeros((len(t_data), 7))
D[:,0] = t_data
D[:,1:] = np.array(f_ext_data)

success_msg = 'success' if success else 'failed'
stamp = time_ns()

filename = f'f_ext_v3_{success_msg}_{stamp}.csv'

if 'save' in sys.argv:
    np.savetxt('data/' + filename, D, delimiter=',')
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
