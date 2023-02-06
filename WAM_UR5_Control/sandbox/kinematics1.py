"""
    mail@kaiploeger.net
"""

import numpy as np
from mujoco_robots.utils import Wam4IK

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from mujoco_robots.robots import MjWam4


def plot_joints(chain, qs):
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1, projection='3d', facecolor="1.0")
    for pos in qs:
        chain.plot([0, 0] + pos + [0, 0, 0], ax)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.show()


def main():
    chain = Wam4IK(base_translation=[0, 0, 0.84],
                   base_orientation=[0, 0, np.pi / 2])

    links = ['wam/links/base',
             'wam/links/shoulder_yaw',
             'wam/links/shoulder_pitch',
             'wam/links/upper_arm',
             'wam/links/forearm',
             'wam/links/tool_base_wo_plate',
             'wam/links/tool_base_w_plate']

    x0 = np.array([0, 0, 0.84])
    q_test = [[0, 0, 0, 0], [1, 1, 1, 1]]

    robot = MjWam4(render=True, g_comp=True)
    for q in q_test:
        print(f'\n\ntesting for q={q}')
        robot.reset(pos=q)
        cart = chain.forward_kinematics([0, 0] + q + [0, 0, 0], full_kinematics=True)

        for i in range(7):
            print(f'\n{links[i][10:]}')
            mj_pos = robot.get_body_full_mat(links[i])[:3, 3] - x0
            ikpy_pos = cart[i + 1][:3, 3] - x0
            print(f'mj:   {mj_pos}')
            print(f'ikpy: {ikpy_pos}')
            print(f'diff: {mj_pos - ikpy_pos}')

    plot_joints(chain, q_test)

    # inverse kinematics
    x_des = [0.15, 0.86, 1.45]
    q = chain.active_from_full(chain.inverse_kinematics(x_des))
    robot.set_mocap_pos('endeff_des', x_des)
    robot.step(des_pos=q, n_steps=5000)


if __name__ == '__main__':
    main()
