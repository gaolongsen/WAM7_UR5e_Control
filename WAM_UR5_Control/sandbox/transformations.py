"""
    A test for:
        - mujoco_robots.robots.MjRobot.get_transform()

    Expected output close to:
    [[ 0.00  1.00  0.00  0.00]
     [-1.00  0.00  0.00  0.00]
     [ 0.00  0.00  0.00 -0.84]
     [ 0.00  0.00  0.00  0.00]]
"""

from mujoco_robots.robots import MjWam4
import numpy as np


def main():
    robot = MjWam4()
    print(robot.get_transform('world', 'wam/links/base'))


if __name__ == '__main__':
    main()
