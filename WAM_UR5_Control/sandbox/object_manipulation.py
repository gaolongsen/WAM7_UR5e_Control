"""
    A test for
        - mujoco_robots.robots.MjRobot.get_object_state()
        - mujoco_robots.robots.MjRobot.set_object_state()

    Expected output:
        [0.1 0.9 1.26]          (position)
        [0. 0. 0.]              (velocity)
        [1. 0. 0. 0.]           (orientation)
        [0. 0. 0.]              (aungular velocity)
"""

import numpy as np
from mujoco_robots.robots import MjWam4


def main():
    robot = MjWam4(xml_path="test.xml")

    pos, vel, quat, vel_euler = robot.get_object_state("ball1", True, True, True, True)
    print(pos, vel, quat, vel_euler, sep='\n')

    robot.set_object_state("ball1", pos, vel, quat, vel_euler)


if __name__ == '__main__':
    main()
