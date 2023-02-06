"""
    A test for the callback based interface of mujoco_robots.robots.MjRobot

    Executes a linear trajectory with the WAM4 and WAM7 closing and opening
    the viewer inbetwwen.
"""

import time
import numpy as np
from mujoco_robots.robots import MjWam7, MjWam4


def main():

    def control_callback(robot):
        t = robot.time
        q0 = np.zeros(robot.n_dof)
        qT = np.ones(robot.n_dof)
        T = 3.
        robot.pos_des = np.minimum(1., (t / T)) * (qT - q0) + q0
        robot.vel_des = np.zeros(robot.n_dof)
        robot.tau_des = np.zeros(robot.n_dof)

    robot = MjWam4()
    robot.set_joint_state(np.zeros(robot.n_dof), np.zeros(robot.n_dof))


    robot.set_control_cb(control_callback)
    robot.start_spinning()
    time.sleep(6.5)
    robot.stop_spinning()

    robot.stop_rendering()
    robot.start_rendering()

    robot.start_spinning()
    time.sleep(4.5)
    robot.stop_spinning()

    # when using a new robot instance make sure to hand over the correct
    # one to the control callback!
    robot.close()
    robot = MjWam7()
    robot.set_joint_state(np.zeros(robot.n_dof), np.zeros(robot.n_dof))
    robot.set_control_cb(control_callback)

    robot.start_spinning()
    time.sleep(3.0)
    robot.stop_spinning()

    robot.close()


if __name__ == '__main__':
    main()
