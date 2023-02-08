# Author: Longsen Gao
# Contact: longsengao@gmail.com

import numpy as np
import threading
from mujoco_robots.robots import MjWam7_pure

if __name__ == "__main__":
    exit_flag = threading.Event()

    # define your controll callback:
    def control_callback(robot):  # the agrument is the MjRobot object
        t = robot.time  # and you can use it's internal time
        q0 = np.zeros(robot.n_dof)
        qT = robot.home_pos
        T = 6.

        if t >= T:  # check whether you are done
            exit_flag.set()

        # Now do whatever you want, but you'll probably want to set some
        # of these variables
        robot.pos_des = np.minimum(1., (t / T)) * (q0 - qT) + qT
        robot.vel_des = np.zeros(robot.n_dof)
        robot.tau_des = np.zeros(robot.n_dof)


    robot = MjWam7_pure(render=True)
    # robot = MjWAM7(render=True)

    # set callback
    robot.set_control_cb(control_callback)

    # execute
    robot.start_spinning()
    exit_flag.wait(10)
    robot.stop_spinning()

    # close the simulation
    robot.close()
