"""
    Author: Longsen Gao
    E-Mail: longsengao@gmail.com
    Affiliate: AgMan Lab, University of New Mexico
    Feel free to contact me if you have any questions for this script.

    A test for mujoco_robots.utils.MjViewerExtended

    The default MjViewer terminates glfw entierly when closing it.
    MjViewerExtended should only close the glfw window s.t. a new one can be
    opened later on.

    Expected behaviour:
        - render WAM4 moving freely and close the viewer
        - sleep 1 second
        - render WAM7 moving freely (with low energy) and close the viewer
        - render WAM4 and WAM7 moving freely and close both viewers
"""

import time
import numpy as np
from mujoco_robots.robots import MjWam4, MjWam7
from mujoco_robots.utils import MjViewerExtended


def main():
    # one viewer opened through __init__()
    robot = MjWam4()
    robot.reset(pos=np.ones(4))
    robot.step(n_steps=1000)
    robot.close()

    time.sleep(1.0)

    # one viewer opened though start_rendering()
    robot = MjWam7(render=True)
    robot.reset(pos=np.zeros(7))
    robot.step(n_steps=1000)
    robot.start_rendering()
    robot.step(n_steps=1000)
    robot.stop_rendering()
    robot.step(n_steps=1000)
    robot.close()

    # two viewers for two robots simultaneously
    robot1 = MjWam4()
    robot1.reset(pos=np.ones(4))
    robot2 = MjWam7()
    robot2.reset(pos=np.ones(7))
    for _ in range(1000):
        robot1.step()
        robot2.step()
    robot1.close()
    robot2.close()


if __name__ == '__main__':
    main()
