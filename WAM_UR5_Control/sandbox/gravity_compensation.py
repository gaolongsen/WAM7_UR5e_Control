"""
    A test for mujoco_robots.robots.GravityCompensationController

    Works if the robots don't not move.
"""

import numpy as np
from mujoco_robots.robots import MjWam4, MjWam7


def main():
    robot = MjWam4(render=True, g_comp=True)
    robot.reset(pos=np.ones(4))
    robot.step(n_steps=1000)
    robot.close()

    robot = MjWam7(render=True, g_comp=True)
    robot.reset(pos=np.ones(7))
    robot.step(n_steps=1000)


if __name__ == '__main__':
    main()
