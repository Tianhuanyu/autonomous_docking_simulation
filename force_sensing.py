import optas
import numpy as np
import pybullet as pb
import matplotlib.pyplot as plt
from time import sleep, time, perf_counter
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
        qF = builder.get_model_state(name, -1)
        TF = robot.get_global_link_transform(ee_link, qF)
        pF = TF[:3, 3]
        zF = TF[:3, 2]
        builder.add_equality_constraint('final_position', pF, goal_position)
        builder.add_equality_constraint('final_axis_align', zF, goal_axis_align)
        builder.enforce_model_limits(name)
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

pb.connect(pb.GUI)
pb.resetSimulation()
pb.setAdditionalSearchPath('resources')

gravz = -9.81
pb.setGravity(0, 0, gravz)

sampling_freq = 240
# sampling_freq = 100
time_step = 1./float(sampling_freq)
pb.setTimeStep(time_step)

initial_position_goal = np.array([0.5, 0., 0.05])

pb.resetDebugVisualizerCamera(
    cameraDistance=1,
    cameraYaw=180,
    cameraPitch=-30,
    cameraTargetPosition=initial_position_goal + np.array([0, 0, 0.1]),
)
pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)

##########################################################
## Load environment

floor_id = pb.createMultiBody(
    baseCollisionShapeIndex=pb.createCollisionShape(shapeType=pb.GEOM_PLANE, planeNormal=[0, 0, 1]),
    baseVisualShapeIndex=pb.createVisualShape(shapeType=pb.GEOM_PLANE, rgbaColor=[0, 1, 0, 1], planeNormal=[0, 0, 1]),
    basePosition=[0, 0, 0],
)

box_height = 0.1
box_width = 0.7

box_id = pb.createMultiBody(
    baseCollisionShapeIndex=pb.createCollisionShape(shapeType=pb.GEOM_BOX, halfExtents=[box_width*0.5, 0.25, box_height*0.5]),
    baseVisualShapeIndex=pb.createVisualShape(shapeType=pb.GEOM_BOX, halfExtents=[box_width*0.5, 0.25, box_height*0.5], rgbaColor=[1, 0., 0., 0.5]),
    basePosition=[0.1, 0, box_height*0.5],
)


##########################################################
## Setup controller

class Controller:

    def __init__(self):
        ee_link = 'storz_tilt_endoscope_link_cm_optical'
        T = 1
        robot = optas.RobotModel(urdf_filename='resources/lbr_with_tilt_endoscope.urdf', time_derivs=[1])
        name = robot.get_name()
        robot.add_base_frame('pybullet_world', [1, 0, 0])
        builder = optas.OptimizationBuilder(T, robots=robot, derivs_align=True)
        qc = builder.add_parameter('qc', robot.ndof)
        dx_goal = builder.add_parameter('dx_goal', 3)
        # q0 = builder.get_model_state(name, 0)
        dq = builder.get_model_state(name, 0, time_deriv=1)
        # qF = builder.get_model_state(name, 1)
        # builder.add_equality_constraint('initial', q0, qc)
        # builder.add_equality_constraint('dynamics', q0 + time_step*dq, qF)
        J = robot.get_global_linear_jacobian(ee_link, qc)
        self.J = robot.get_global_linear_jacobian_function(ee_link)
        dx = J@dq
        builder.add_cost_term('eff_vel', optas.sumsqr(dx - dx_goal))

        # y = robot.get_global_link_position(ee_link, qc)[1]
        # yF = y + time_step*dx[1]

        # builder.add_cost_term('eff_y', optas.sumsqr(yF))
        # builder.enforce_model_limits(name)
        builder.add_cost_term('min_joint_vel', 1e-6*optas.sumsqr(dq))
        lo = robot.lower_actuated_joint_limits
        up = robot.upper_actuated_joint_limits
        qF = qc + time_step*dq
        builder.add_bound_inequality_constraint(name, lo, qF, up)
        self.solver = optas.OSQPSolver(builder.build()).setup(use_warm_start=True)
        # self.solver = optas.CasADiSolver(builder.build()).setup('ipopt')
        self.solution = None
        self.name = name

    def __call__(self, qc, dx_goal):
        if self.solution is not None:
            self.solver.reset_initial_seed(self.solution)
        else:
            self.solver.reset_initial_seed({f'{self.name}/q': optas.horzcat(qc, qc)})
        self.solver.reset_parameters({'qc': qc, 'dx_goal': dx_goal})
        t0 = perf_counter()
        self.solution = self.solver.solve()
        t1 = perf_counter()
        solver_duration=t1 -t0
        print("-------------")
        print(f"{solver_duration=}, {1./solver_duration=}")
        print("-------------")
        dq_goal = self.solution[f'{self.name}/dq'].toarray().flatten()
        return qc + time_step*dq_goal, dq_goal

controller = Controller()

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
        lim = 9*np.ones(7)

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
## Move to initial configuration

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
        robot.cmd(plan(dur))
        pb.stepSimulation()
        sleep(time_step*time_factor)
        robot.update()

p0 = initial_position_goal
z0 = np.array([0., 0., -1.])
dur = 4.
initial_plan = planner(robot.q(), p0, z0, dur, np.zeros(3))
exec_plan(initial_plan, dur, time_factor=0.001)

print(robot._Tf(initial_plan(dur)).toarray())
print(robot.Tf())

##########################################################
## Keep simulator running

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

dx_goal = [-0.02, 0, 0]

p = robot.p()

time_factor = 0.0000

q = initial_plan(dur)

t = 0.

def norm(a, b=0.):
    return np.linalg.norm(a-b)

peff_data = []
response_data = []
t_data = []
f_ext_lin_data = []

while pb.isConnected():

    t_data.append(t)

    # q_0 = robot.q()
    # q_1 = robot.q()

    # print("-------------")
    # print(f"{norm(q_0, q_1)=}")
    # print("-------------")

    f_ext = robot.f_ext()
    f_ext_lin = f_ext[:3]

    f_ext_lin_data.append(f_ext_lin.copy().tolist())

    peff = robot.p()
    peff_data.append(peff.copy().tolist())
    print("-------------")
    print(f"{f_ext_lin=}")
    print("-------------")
    visualize_line(peff, peff + f_ext_lin)

    response = 0.3*f_ext_lin
    response_data.append(response.copy().tolist())

    print("-------------")
    print(f"{response=}")
    print("-------------")

    dx_goal_use = dx_goal + response

    print("-------------")
    print(f"{dx_goal_use=}")
    print("-------------")

    q, dq_goal = controller(q, dx_goal_use)
    dp_ideal = controller.J(robot.q()).toarray()@dq_goal
    # print("-------------")
    # print(f"{dq_goal=}")
    # print("-------------")
    # print("-------------")
    # print(f"{dp_ideal=}")
    # print("-------------")
    # if t < 1:
    robot.cmd(q)
    # robot.cmd_vel(dq_goal)
    p_new = robot.p()
    dp = (p_new - p)/time_step
    print("-------------")
    print(f"{dp=}")
    print("-------------")
    p = p_new.copy()
    pb.stepSimulation()
    sleep(time_step*time_factor)

    t += time_step

    if t > 5.:
        break

pb.disconnect()

t = np.array(t_data)
rx = np.array([r[0] for r in response_data])
px = np.array([p[0] for p in peff_data])
# fx = np.array([f[0] for f in f_ext_lin_data])

fig, ax = plt.subplots(
    2, # 3,
    1,
    sharex=True,
)

ax[0].plot(t, rx, label='x')
ax[1].plot(t, px, label='x')
# ax[2].plot(t, fx, label='x')

ax[0].set_ylabel('response')
ax[1].set_ylabel('eff')
# ax[2].set_ylabel('f_ext')

for a in ax.flatten():
    a.grid()

plt.show()
