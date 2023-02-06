# Author: Longsen Gao
# Contact: longsengao@gmail.com
# Created Time： 11-27-2022

import os
import inspect
import time
import mujoco_py
import numpy as np
import pandas as pd

from threading import Thread
# from mujoco_robots import transform
from mujoco_robots.utils import MjViewerExtended


def test_sensors():
    model = mujoco_py.load_model_from_path("/home/wam/mujoco_robots/robot_description/wam7/wam_7dof.xml")
    sim = mujoco_py.MjSim(model)
    sensor = np.zeros(7)
    sensor[0] = sim.data.get_sensor("fsensor3_6_1")
    sensor[1] = sim.data.get_sensor("fsensor3_6_2")
    sensor[3] = sim.data.get_sensor("fsensor3_6_3")
    sensor[4] = sim.data.get_sensor("fsensor3_7_1")
    sensor[5] = sim.data.get_sensor("fsensor3_7_2")
    sensor[6] = sim.data.get_sensor("fsensor3_7_3")
    return sensor


class MjRobot():
    """ An abstract base class to wrap mujoco_py simulations """

    def __init__(self, xml_path, object_names=[], render=True,
                 g_comp=False, tool_mass=0, tool_mass_site=None):
        """ An abstract base class to wrap mujoco_py simulations """
        assert xml_path is None or os.path.isfile(xml_path)
        assert hasattr(self, 'n_dof')  # Int
        assert hasattr(self, 'home_pos')  # numpy.ndarray
        assert hasattr(self, 'p_gains')  # numpy.ndarray
        assert hasattr(self, 'd_gains')  # numpy.ndarray
        assert hasattr(self, 'max_ctrl')  # numpy.ndarray
        assert hasattr(self, 'min_ctrl')  # numpy.ndarray
        assert hasattr(self, 'dt')  # Float
        assert not ((tool_mass == 0) ^ (tool_mass_site is None))  # must set both

        self.model = mujoco_py.load_model_from_path(xml_path)
        self.sim = mujoco_py.MjSim(self.model, nsubsteps=1)
        self.viewer = None

        if g_comp:
            self.g_comp_ctl = GravityCompensationController(self)
        else:
            self.g_comp_ctl = None
        if tool_mass_site is not None:
            self.mass_names.append(tool_mass_site)
            self.masses = np.append(self.masses, tool_mass)

        # data recording
        self.object_names = object_names
        self.rendering = render
        self.recording = False
        self.video_recording = False
        self._recorded_trajectory = []

        self.reset()
        self.wait()

    def close(self):
        """ closes all open threads """
        self.stop_spinning()
        self.stop_rendering()

    # ========== basic simulation control ==========

    def step(self, des_pos=None, des_vel=None, tau=None, n_steps=1):
        """ advances the simulation by one time step """
        assert des_pos is None or len(des_pos) == self.n_dof and isinstance(des_pos, np.ndarray)
        assert des_vel is None or len(des_vel) == self.n_dof and isinstance(des_vel, np.ndarray)
        assert tau is None or len(tau) == self.n_dof and isinstance(tau, np.ndarray)

        for _ in range(n_steps):
            motor_torques = np.zeros(self.n_dof)
            # ...pos if available and 0 vel if no vel availavle
            if not des_pos is None:
                motor_torques += self.p_gains * (des_pos.ravel() - self.pos)
                if des_vel is None:  # damping to prevent P control only
                    motor_torques -= self.d_gains * self.vel
            if not des_vel is None:
                motor_torques += self.d_gains * (des_vel.ravel() - self.vel)
            if not tau is None:
                motor_torques += tau.ravel()
            if self.g_comp_ctl is not None:
                tau_g_comp = self.g_comp_ctl()
                motor_torques += tau_g_comp
            motor_torques = np.maximum(np.minimum(motor_torques, self.max_ctrl), self.min_ctrl)

            # apply actions and advance the simulation
            self.sim.data.qfrc_applied[:self.n_dof] = motor_torques
            self.sim.step()

            # get sim state / observations
            self.pos = self.sim.data.qpos[:self.n_dof].copy()
            self.vel = self.sim.data.qvel[:self.n_dof].copy()

            # update time
            self.time = self.sim.data.time
            self.timestep += 1
            self.time_task += self.dt
            self.timestep_task += 1

            # render...
            if self.rendering:
                if self.viewer is None:  # ...and setup a viewer if not available
                    self._make_viewer()
                self.viewer.render()

            if self.recording:
                self.record_current_time_step()

        return self.pos, self.vel, self.time

    def reset(self, pos=None, vel=None, task_name=''):
        """ reset the simulation state
            task_name is an optional label for data recording
        """
        assert pos is None or len(pos) == self.n_dof
        assert vel is None or len(vel) == self.n_dof
        self.sim.reset()

        self.time = self.sim.data.time
        self.timestep = 0.
        self.timestep_task = 0.
        self.time_task = 0.
        self.task_name = task_name
        self.task_done = False

        if pos is None:
            self.sim.data.qpos[:self.n_dof] = self.home_pos
        else:
            self.sim.data.qpos[:self.n_dof] = np.array(pos)
        if vel is None:
            self.sim.data.qvel[:self.n_dof] = np.zeros(self.n_dof)
        else:
            self.sim.data.qvel[:self.n_dof] = np.array(vel)
        self.pos = self.sim.data.qpos[:self.n_dof].copy()
        self.vel = self.sim.data.qvel[:self.n_dof].copy()
        self.pos_des = self.sim.data.qpos[:self.n_dof].copy()
        self.vel_des = self.sim.data.qvel[:self.n_dof].copy()
        self.tau_des = np.zeros(self.n_dof)  # feed forward

        self.sim.forward()

        return self.pos, self.vel, self.time

    def get_joint_state(self, pos=True, vel=False):
        """ returns joint positions and velocities if requested """
        ret = []
        if pos is True:
            ret.append(self.sim.data.qpos[:self.n_dof])
        if vel is True:
            ret.append(self.sim.data.qvel[:self.n_dof])
        if len(ret) == 1:
            return ret[0]
        return tuple(ret)

    def set_joint_state(self, pos=None, vel=None):
        """ sets the positions and velocities of all robot joints
            think about using reset instead
        """
        if pos is not None:
            self.sim.data.qpos[:self.n_dof] = pos
        if vel is not None:
            self.sim.data.qvel[:self.n_dof] = vel

    def set_object_state(self, name, pos=None, vel=None, quat=None, vel_euler=None):
        """ overwrites all provided state variables of the free joint
            named name - this sets the state of the corresponding object wrt
            the frame the free joint is defined in (usually world)
        """
        x = self.sim.data.get_joint_qpos(name)
        dx = self.sim.data.get_joint_qvel(name)
        if pos is not None:
            x[0:3] = pos
        if vel is not None:
            dx[0:3] = vel
        if quat is not None:
            x[3:7] = quat
        if vel_euler is not None:
            dx[3:6] = vel_euler
        self.sim.data.set_joint_qpos(name, x)
        self.sim.data.set_joint_qvel(name, dx)

    def get_object_state(self, name, pos=True, vel=False, quat=False, vel_euler=False):
        """ returns all requested state variables of the free joint
            named name, which are identical to the state of the corresponding
            object wrt the frame the free joint is defined in (usually world)
        """
        x = self.sim.data.get_joint_qpos(name)
        dx = self.sim.data.get_joint_qvel(name)
        ret = []
        if pos is True:
            ret.append(x[0:3])
        if vel is True:
            ret.append(dx[0:3])
        if quat is True:
            ret.append(x[3:7])
        if vel_euler is True:
            ret.append(dx[3:6])
        if len(ret) == 1:
            return ret[0]
        return tuple(ret)

    def get_site_state(self, name, pos=True, vel=False, xmat=False, vel_euler=False):
        """ returns all requested state variables of the site named name """
        ret = []
        if pos is True:
            ret.append(self.sim.data.get_site_xpos(name))
        if vel is True:
            ret.append(self.sim.data.get_site_xvelp(name))
        if xmat is True:
            ret.append(self.sim.data.get_site_xmat(name))
        if vel_euler is True:
            ret.append(self.sim.get_site_xvelr(name))
            MjViewerExtended.terminate_all()
        if len(ret) == 1:
            return ret[0]
        return tuple(ret)

    def get_transform(self, from_body, to_body):
        """ returns the homogeneous transformation from one body defined in the
            xml description to another one as 4x4 matrix """
        from_world = self.get_body_full_mat(from_body)
        to_world = self.get_body_full_mat(to_body)
        world_to = np.linalg.inv(to_world)
        trafo = from_world @ world_to
        return trafo

    def get_body_full_mat(self, body):
        """ returns the homogeneous transformation from a body defined in the
            xml description to the world frame as 4x4 matrix
        """
        if body == 'world':
            rot = np.eye(3)
            pos = np.zeros((3, 1))
        else:
            rot = self.sim.data.get_body_xmat(body)
            pos = self.sim.data.get_body_xpos(body).reshape((3, 1))
        upper_mat = np.hstack([rot, pos])
        full_mat = np.vstack([upper_mat, np.array([[0, 0, 0, 1]])])
        return full_mat

    # ========== visuals and recording ==========

    def start_rendering(self):
        self.rendering = True

    def stop_rendering(self):
        """ closes the current viewer """
        if self.rendering:
            self.rendering = False
            self.viewer.close()
            self.viewer = None

    def _make_viewer(self):
        """ inherit to configure your own defaults
            more options:
            self.viewer.cam.distance = self.model.stat.extent * 1.0   # zoom
            self.viewer.cam.trackbodyid = 0    # id of the body to track ()
            self.viewer.cam.lookat[0] += 0.5   # works if trackbodyid=-1
            self.viewer.cam.lookat[1] += 0.5
            self.viewer.cam.lookat[2] += 0.5
            self.viewer.cam.elevation = -90    # camera tilting
            self.viewer.cam.azimuth = 0        # camera panning
        """
        self.viewer = MjViewerExtended(self.sim)
        self.viewer._hide_overlay = True
        self.viewer._run_speed = 1.0
        self.rendering = True

    def set_mocap_pos(self, mocap_name, pos):
        """ sets the position of a specified mocap object_names in world coords
            this is useful to show things like desired cartesian positions
        """
        self.sim.data.set_mocap_pos(mocap_name, pos)

    def start_data_recording(self):
        """ start recording the simulation state each time step """
        self.recording = True

    def stop_data_recording(self):
        """ stop recording the simulation state """
        self.recording = False

    def get_data_recording(self):
        """ return a DataFrame of the recorded robot and object states """
        q_des = [f"{joint_name}_des" for joint_name in self.joint_names]
        dq_des = [f"d{joint_name}_des" for joint_name in self.joint_names]
        q = [f"{joint_name}" for joint_name in self.joint_names]
        dq = [f"d{joint_name}" for joint_name in self.joint_names]

        value_types = ['pos_x', 'pos_y', 'pos_z', 'quat_x', 'quat_y', 'quat_z', 'quat_w']
        objs = [f"{val_type}_{name}" for name in self.object_names for val_type in value_types]

        columns = ['time', 'time_task', 'task'] + q_des + dq_des + q + dq + objs

        df = pd.DataFrame(self._recorded_trajectory[:], columns=columns)
        return df

    def clear_data_recording(self):
        """ clears all recorded data from cache """
        self._recorded_trajectory = []

    def record_current_time_step(self):
        """ adds the current simulation state to a buffer
            is called each time step if self.recording is set
        """
        obj_states = []
        for obj in range(len(self.object_names)):
            obj_states += self.obj_pos[obj].tolist()
            obj_states += self.obj_quat[obj].tolist()
        self._recorded_trajectory.append([self.time, self.time_task, self.task_name]
                                         + self.pos_des.tolist()
                                         + self.vel_des.tolist()
                                         + self.pos.tolist()
                                         + self.vel.tolist()
                                         + obj_states)

    def start_video_recording(self, cameras, video_subsampling=10):
        """ start recording a video of the simulation """
        raise NotImplementedError("Video recording does not work yet")
        self.cameras = cameras
        self.video_subsampling = video_subsampling
        self._recorded_video = [[] for _ in range(len(cameras))]
        self.video_recording = True

    def stop_video_recording(self):
        """ stop recording a video of the simulation """
        self.video_recording = False

    def get_image(self, camera_name, width=128, height=128, depth=False):
        """ returns the view of a given camera specified in the xml file """
        img = self.sim.render(camera_name=camera_name, width=width, height=height, depth=depth)
        return img

    # ========== callback based control ==========
    # can be used similar to Robcom 2

    def set_control_cb(self, cb, duration=-1, task_name=''):
        """ replaces the current control callback

            cb: use the callback defined in goto_joint_cubic as template
            duration: in time steps or -1 for infinite
            task_name: an optional label for data recording """
        self.time_task = 0
        self.timestep_task = 0
        self.task_duration = duration
        self.task_name = task_name
        self.control_callback = cb

    def start_spinning(self, sync_mode=False):
        """ starts a thread alternating between calling the control callback
            and step
            will be approximately real-time when rendering is on

            sync_mode: only here for compatibility with robocom_robots"""

        def spin():
            while self._spinning:
                if not self.control_callback is None:
                    self.control_callback(self)
                    self.step(self.pos_des, self.vel_des, self.tau_des)

        self._spinning = True
        self.control_thread = Thread(target=spin)
        self.control_thread.start()

    def stop_spinning(self):
        """ stops the thread started in start_spinning """
        if hasattr(self, "_spinning") and self._spinning:
            self._spinning = False
            self.control_thread.join()

    # ========== task based control ==========
    # very simple to use - similar to the goto actions from ias_ros

    def wait_for_task(self, dt=0.1):
        """ sleeps in dt time increments until the current task is done """
        while not self.task_done:
            time.sleep(dt)

    def wait(self, task_name='WAIT'):
        """ holds current position """
        self.task_name = task_name
        self.vel_des = np.zeros(self.n_dof)

        cb = lambda robot: None  # do nothing
        self.set_control_cb(cb)

    def goto_joint_cubic(self, q, dq=None, T=5, task_name='GOTO_CUBIC'):
        """ moves the robot to the desired position, then waits
            q: target joint position in rad
            dq: target joint angular velocity in rad/s
            T: target duration in sec """
        self.task_done = False
        self.task_name = task_name

        q_0 = self.pos_des
        dq_0 = self.vel_des
        q_T = np.array(q)
        if dq is None:
            dq_T = np.array(self.n_dof)
        else:
            dq_T = np.array(dq)
        N = float(T) / self.dt

        a = - 2 * (q_T - q_0) / N ** 3 + (dq_T + dq_0) / N ** 2
        b = 3 * (q_T - q_0) / N ** 2 - (dq_T + 2 * dq_0) / N
        c = dq_0
        d = q_0

        def goto_joint_cubic_cb(robot, a, b, c, d):
            t = robot.timestep_task
            robot.pos_des = a * t ** 3 + b * t ** 2 + c * t + d
            robot.vel_des = (3 * a * t ** 2 + 2 * b * t + c) / robot.dt
            robot.tau_des = None
            if robot.timestep_task >= robot.task_duration:
                robot.task_done = True
                robot.wait(task_name='WAIT_AFTER_' + robot.task_name)

        cb = lambda robot: goto_joint_cubic_cb(robot, a, b, c, d)
        self.set_control_cb(cb, N)

    def goto_home(self, duration=4.):
        """ moves the robot to it's home position, then waits"""
        self.goto_joint_cubic(self.home_pos, np.zeros(self.n_dof), duration, task_name='GO_HOME')


class HFinger1(MjRobot):
    default_xml_file = "wam7/wam_7dof.xml"
    n_dof = 3
    home_pos = np.array([0., 0., 0.0])
    p_gains = np.array([300, 200.0, 300.0])
    d_gains = np.array([20.0, 7.0, 15.0])
    max_ctrl = np.array([3.14, 2.44, 0.83])
    min_ctrl = np.array([0, 0, 0])
    dt = 0.001

    joint_names = [f'q{i}' for i in range(1, n_dof + 1)]
    masses = np.array([
        0.14109,
        0.062139,
        0.041377])  # same as in xml file, for gravity compensation
    mass_names = [
        "wam/mass_sites/finger_1/prox_link",
        "wam/mass_sites/finger_1/med_link",
        "wam/mass_sites/finger_1/dist_link"
    ]

    def __init__(self, xml_path=None, object_names=[], render=True,
                 g_comp=False, tool_mass=0, tool_mass_site=None):
        """ The 7 DoF, 80V Barret WAM robot
        xml_path: to change the robots environment or end effector, provide a
                  modified version of the default xml description file
        object_names: states of the listed objects are included in recordings
        render: whether or not to render the simulation
        g_comp: whether or not to use gravity compensation """

        if xml_path == None:
            script_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
            xml_path = script_path + '/../robot_description/' + self.default_xml_file

        MjRobot.__init__(self, xml_path, object_names=object_names, render=render,
                         g_comp=g_comp, tool_mass=tool_mass, tool_mass_site=tool_mass_site)

        self.ROBOT2WORLD = self.get_body_full_mat('wam/bhand/finger_1/prox_link')
        self.WORLD2ROBOT = self.get_transform('wam/bhand/finger_1/prox_link', 'wam/bhand/finger_1/dist_link')


class UR5_pure(MjRobot):
    """ The 6 DoF, 80V UR5 WAM robot """
    # default_xml_file = "wam7_Pro/wam_7dof.xml"
    default_xml_file = "wam7/wam_7_Pure_UR5.xml"

    # robot properties
    # n_dof = 11
    n_dof = 6

    home_pos = np.array([0.8164, -1.5386, -1.6014, 0, 0, 0])
    p_gains = np.array([300.0, 150.0, 50.0, 50.0, 50.0, 10.0])
    d_gains = np.array([15.0, 15.0, 10.0, 7.0, 5.3, 5.2])
    max_ctrl = np.array([360.0, 360.0, 360.0, 360.0, 360.0, 360.0])
    min_ctrl = np.array(
        [-360.0, -360.0, -360.0, -360.0, -360.0, -360.0])

    # home_pos = np.array([0., -1.986, 0.0, 3.146, 0.0, 0.0, 0.0])
    # p_gains = np.array([200.0, 300.0, 200.0, 100.0, 100.0, 10.0, 2.5])
    # max_ctrl = np.array([150.0, 113.0, 157.0, 180.0, 75.0, 90.0, 128.0])
    # min_ctrl = np.array([-150.0, -113.0, -157.0, -50.0, -275.0, -90.0, -128.0])

    dt = 0.001

    joint_names = [f'q{i}' for i in range(1, n_dof + 1)]
    masses = np.array([
        3.7,
        8.393,
        2.275,
        1.219,
        1.219,
        0.1879
    ])  # same as in xml file, for gravity compensation
    mass_names = [
        "ur5/mass_sites/shoulder_link",
        "ur5/mass_sites/upper_arm_link",
        "ur5/mass_sites/forearm_link",
        "ur5/mass_sites/wrist1_link",
        "ur5/mass_sites/wrist2_link",
        "ur5/mass_sites/wrist3_link"
    ]

    def __init__(self, xml_path=None, object_names=[], render=True,
                 g_comp=False, tool_mass=0, tool_mass_site=None):
        """ The 6 DoF, 80V UR5e robot
        xml_path: to change the robots environment or end effector, provide a
                  modified version of the default xml description file
        object_names: states of the listed objects are included in recordings
        render: whether or not to render the simulation
        g_comp: whether or not to use gravity compensation """

        if xml_path == None:
            script_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
            xml_path = script_path + '/../robot_description/' + self.default_xml_file

        MjRobot.__init__(self, xml_path, object_names=object_names, render=render,
                         g_comp=g_comp, tool_mass=tool_mass, tool_mass_site=tool_mass_site)

        # transformations:
        # self.ROBOT2WORLD = self.get_body_full_mat('wam/links/Track')
        # self.WORLD2ROBOT = self.get_transform('world', 'wam/links/Track')

        self.ROBOT2WORLD = self.get_body_full_mat('base_link')
        self.WORLD2ROBOT = self.get_transform('base_link', 'wrist3_link')

    def _make_viewer(self):
        """ visuals of the simulation, see MjRobot.viewer_setup() """
        MjRobot._make_viewer(self)
        self.viewer.cam.distance = 2.5
        self.viewer.cam.lookat[0] += 0.15
        self.viewer.cam.elevation = -25
        self.viewer.cam.azimuth = -90


class MjWam7_pure(MjRobot):
    """ The 7 DoF, 80V Barret WAM robot """
    # default_xml_file = "wam7_Pro/wam_7dof.xml"
    default_xml_file = "wam7/wam_7_Pure_UR5.xml"

    # robot properties
    # n_dof = 11
    n_dof = 7

    home_pos = np.array([0., -1.986, 0.0, 3.146, 0.0, -1.57, 0.0])
    p_gains = np.array([300.0, 150.0, 50.0, 50.0, 50.0, 10.0, 2.5])
    d_gains = np.array([15.0, 15.0, 10.0, 7.0, 5.3, 5.2, 2.05])
    max_ctrl = np.array([150.0, 113.0, 157.0, 180.0, 75.0, 90.0, 128.0])
    min_ctrl = np.array(
        [-150.0, -113.0, -157.0, -50.0, -275.0, -90.0, -128.0])

    # home_pos = np.array([0., -1.986, 0.0, 3.146, 0.0, 0.0, 0.0])
    # p_gains = np.array([200.0, 300.0, 200.0, 100.0, 100.0, 10.0, 2.5])
    # max_ctrl = np.array([150.0, 113.0, 157.0, 180.0, 75.0, 90.0, 128.0])
    # min_ctrl = np.array([-150.0, -113.0, -157.0, -50.0, -275.0, -90.0, -128.0])

    dt = 0.001

    joint_names = [f'q{i}' for i in range(1, n_dof + 1)]
    masses = np.array([
        10.76768767,
        3.87493756,
        1.80228141,
        2.40016804,
        0.12376019,
        0.41797364,
        0.06864753
    ])  # same as in xml file, for gravity compensation
    mass_names = [
        "wam/mass_sites/shoulder_yaw",
        "wam/mass_sites/shoulder_pitch",
        "wam/mass_sites/upper_arm",
        "wam/mass_sites/forearm",
        "wam/mass_sites/wrist_yaw",
        "wam/mass_sites/wrist_pitch",
        "wam/mass_sites/wrist_palm"
    ]

    def __init__(self, xml_path=None, object_names=[], render=True,
                 g_comp=False, tool_mass=0, tool_mass_site=None):
        """ The 7 DoF, 80V Barret WAM robot
        xml_path: to change the robots environment or end effector, provide a
                  modified version of the default xml description file
        object_names: states of the listed objects are included in recordings
        render: whether or not to render the simulation
        g_comp: whether or not to use gravity compensation """

        if xml_path == None:
            script_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
            xml_path = script_path + '/../robot_description/' + self.default_xml_file

        MjRobot.__init__(self, xml_path, object_names=object_names, render=render,
                         g_comp=g_comp, tool_mass=tool_mass, tool_mass_site=tool_mass_site)

        # transformations:
        # self.ROBOT2WORLD = self.get_body_full_mat('wam/links/Track')
        # self.WORLD2ROBOT = self.get_transform('world', 'wam/links/Track')

        self.ROBOT2WORLD = self.get_body_full_mat('wam/links/base')
        self.WORLD2ROBOT = self.get_transform('wam/links/base', 'wam/links/wrist_palm')

    def _make_viewer(self):
        """ visuals of the simulation, see MjRobot.viewer_setup() """
        MjRobot._make_viewer(self)
        self.viewer.cam.distance = 2.5
        self.viewer.cam.lookat[0] += 0.15
        self.viewer.cam.elevation = -25
        self.viewer.cam.azimuth = -90


class MjWam7(MjRobot):
    """ The 7 DoF, 80V Barret WAM robot """
    # default_xml_file = "wam7_Pro/wam_7dof.xml"
    default_xml_file = "wam7/wam_7_UR5.xml"

    # robot properties
    # n_dof = 11
    n_dof = 15

    home_pos = np.array([0., -1.986, 0.0, 3.146, 0.0, -1.57, 0.0, 2.33, 0.84, 0.0, 2.33, 0.84, 0, 2.33, 0.84])
    p_gains = np.array([300.0, 150.0, 50.0, 50.0, 50.0, 10.0, 2.5, 1, 1, 1, 1, 1, 1, 1, 1])
    d_gains = np.array([15.0, 15.0, 10.0, 7.0, 5.3, 5.2, 2.05, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02])
    max_ctrl = np.array([150.0, 113.0, 157.0, 180.0, 75.0, 90.0, 128.0, 139, 48, 180, 139, 48, 180, 139, 48])
    min_ctrl = np.array(
        [-150.0, -113.0, -157.0, -50.0, -275.0, -90.0, -128.0, -139, -48, 0, -139, -48, 0, -139, -48])

    # home_pos = np.array([0., -1.986, 0.0, 3.146, 0.0, 0.0, 0.0])
    # p_gains = np.array([200.0, 300.0, 200.0, 100.0, 100.0, 10.0, 2.5])
    # max_ctrl = np.array([150.0, 113.0, 157.0, 180.0, 75.0, 90.0, 128.0])
    # min_ctrl = np.array([-150.0, -113.0, -157.0, -50.0, -275.0, -90.0, -128.0])

    dt = 0.001

    joint_names = [f'q{i}' for i in range(1, n_dof + 1)]
    masses = np.array([
        10.76768767,
        3.87493756,
        1.80228141,
        2.40016804,
        0.12376019,
        0.41797364,
        0.06864753,
        0.50573,
        0.062139,
        0.041377,
        0.14109,
        0.062139,
        0.041377,
        0.14109,
        0.062139,
        0.041377
    ])  # same as in xml file, for gravity compensation
    mass_names = [
        "wam/mass_sites/shoulder_yaw",
        "wam/mass_sites/shoulder_pitch",
        "wam/mass_sites/upper_arm",
        "wam/mass_sites/forearm",
        "wam/mass_sites/wrist_yaw",
        "wam/mass_sites/wrist_pitch",
        "wam/mass_sites/wrist_palm",
        "wam/mass_sites/bhand_palm_link",
        "wam/mass_sites/finger_3/med_link",
        "wam/mass_sites/finger_3/dist_link",
        "wam/mass_sites/finger_1/prox_link",
        "wam/mass_sites/finger_1/med_link",
        "wam/mass_sites/finger_1/dist_link",
        "wam/mass_sites/finger_2/prox_link",
        "wam/mass_sites/finger_2/med_link",
        "wam/mass_sites/finger_2/dist_link"
    ]

    def __init__(self, xml_path=None, object_names=[], render=True,
                 g_comp=False, tool_mass=0, tool_mass_site=None):
        """ The 7 DoF, 80V Barret WAM robot
        xml_path: to change the robots environment or end effector, provide a
                  modified version of the default xml description file
        object_names: states of the listed objects are included in recordings
        render: whether or not to render the simulation
        g_comp: whether or not to use gravity compensation """

        if xml_path == None:
            script_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
            xml_path = script_path + '/../robot_description/' + self.default_xml_file

        MjRobot.__init__(self, xml_path, object_names=object_names, render=render,
                         g_comp=g_comp, tool_mass=tool_mass, tool_mass_site=tool_mass_site)

        # transformations:
        # self.ROBOT2WORLD = self.get_body_full_mat('wam/links/Track')
        # self.WORLD2ROBOT = self.get_transform('world', 'wam/links/Track')

        self.ROBOT2WORLD = self.get_body_full_mat('wam/links/base')
        self.WORLD2ROBOT = self.get_transform('wam/links/base', 'wam/bhand/bhand_palm_link')

    def _make_viewer(self):
        """ visuals of the simulation, see MjRobot.viewer_setup() """
        MjRobot._make_viewer(self)
        self.viewer.cam.distance = 2.5
        self.viewer.cam.lookat[0] += 0.15
        self.viewer.cam.elevation = -25
        self.viewer.cam.azimuth = -90


class MjWam4(MjRobot):
    """ The 4 DoF, 80V Barret WAM robot """
    default_xml_file = "wam4/wam_4dof.xml"

    # robot properties
    n_dof = 4
    home_pos = np.array([0., -1.986, 0., 3.146])
    p_gains = np.array([200.0, 300.0, 100.0, 100.0])
    d_gains = np.array([7.0, 15.0, 5.0, 2.5])
    max_ctrl = np.array([150.0, 125.0, 40.0, 60.0])
    min_ctrl = -max_ctrl
    dt = 0.002

    joint_names = [f'q{i}' for i in range(1, n_dof + 1)]
    masses = np.array([10.76768767,
                       3.87493756,
                       1.80228141,
                       1.06513649])  # same as in xml file, for gravity compensation
    mass_names = ["wam/mass_sites/shoulder_yaw",
                  "wam/mass_sites/shoulder_pitch",
                  "wam/mass_sites/upper_arm",
                  "wam/mass_sites/forearm"]

    def __init__(self, xml_path=None, object_names=[], render=True,
                 g_comp=False, tool_mass=0, tool_mass_site=None):
        """ The 4 DoF, 80V Barret WAM robot
        xml_path: to change the robots environment or end effector, provide a
                  modified version of the default xml description file
        object_names: states of the listed objects are included in recordings
        render: whether or not to render the simulation
        g_comp: whether or not to use gravity compensation """

        if xml_path == None:
            script_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
            xml_path = script_path + '/../robot_description/' + self.default_xml_file

        MjRobot.__init__(self, xml_path, object_names=object_names, render=render,
                         g_comp=g_comp, tool_mass=tool_mass, tool_mass_site=tool_mass_site)

        # transformations:
        self.ROBOT2WORLD = self.get_body_full_mat('wam/links/base')
        self.WORLD2ROBOT = self.get_transform('world', 'wam/links/base')

    def _make_viewer(self):
        """ visuals of the simulation, see MjRobot.viewer_setup() """
        MjRobot._make_viewer(self)
        self.viewer.cam.azimuth = -45
        self.viewer.cam.distance = 2.5
        self.viewer.cam.lookat[0] += 0.15
        self.viewer.cam.lookat[2] += 0.3
        self.viewer.cam.elevation = -25
        self.viewer.cam.azimuth = -90


class GravityCompensationController():
    """ A simple gravity compensation controller for any MjRobot

        Requires the MjRobot to have a 'site' object defined at every mass
        location in the xml description and a list of their names
        (MjRobot.mass_names) to compute the Jacobians """

    def __init__(self, robot, gravity=np.array([0., 0., -9.81])):
        """ A simple gravity compensation controller for any MjRobot """
        self.robot = robot
        self.g = gravity

    def __call__(self):
        """ returns the torques that currently compensate gravity """
        tau = np.zeros(self.robot.n_dof)
        for i in range(len(self.robot.mass_names)):
            # get jacobian
            target_jacp = np.zeros(3 * self.robot.sim.model.nv)
            self.robot.sim.data.get_site_jacp(self.robot.mass_names[i], jacp=target_jacp)

            # get rid of additional objects's free joints in target_jacp
            jac = np.array(target_jacp).reshape((3, self.robot.sim.model.nv))[:, :self.robot.n_dof]

            # compute torques with jacobian transpose method
            tau += jac.T @ (- self.robot.masses[i] * self.g)

        return tau
