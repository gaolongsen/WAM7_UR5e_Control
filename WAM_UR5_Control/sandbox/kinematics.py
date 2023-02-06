"""
    longsengao@gmail.com
"""
import pinocchio as pin
from pinocchio.utils import *

import numpy as np
from mujoco_robots.utils import Wam7IK
import time
import matplotlib.pyplot as plt
import os

from mpl_toolkits import mplot3d

# from juggling_wams.envs import SingleArmOneBall
from mujoco_robots.robots import MjWam7, HFinger1


def plot_joints(chain, x_des):
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1, projection='3d', facecolor="1.0")

    chain.plot(chain.inverse_kinematics(x_des), ax)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.show()


def main():
    chain = Wam7IK(
        # base_translation=[-1.5, 0, 0],
        # base_orientation=[0, 0, np.pi / 2]
        base_translation=[0., 0., 0.84],
        base_orientation=[0, 0, np.pi / 2]
    )

    links = [
        # 'wam/links/Track',
        'wam/links/base',
        'wam/links/shoulder_yaw',
        'wam/links/shoulder_pitch',
        'wam/links/upper_arm',
        'wam/links/forearm',
        'wam/links/wrist_yaw',
        'wam/links/wrist_pitch',
        'wam/links/wrist_palm',
        "wam/bhand/bhand_palm_link",
        "wam/bhand/finger_3/med_link",
        "wam/bhand/finger_3/dist_link"
    ]

    goal_pos_f1 = [2, 0.1, 0.8]
    goal_vel_f1 = np.zeros(3)

    # x0 = np.array([-1.5, 0, 0])
    x0 = np.array([0, 0, 0.84])
    q_test = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [1., 0., -1.986, 0.0, 3.146, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
    # q_test = [[0, 0, 0, 0, 0, 0, 0], [0., -1.986, 0.0, 3.146, 0.0, 0.0, 0.0]]
    robot = MjWam7(render=True, g_comp=True)
    model = pin.Model
    finger1 = HFinger1(render=True, g_comp=True)
    for q in q_test:
        print(f'\n\ntesting for q={q}')
        robot.reset(pos=q)
        cart = chain.forward_kinematics([0] + q, full_kinematics=True)

        for i in range(11):
            print(f'\n{links[i][10:]}')  # print each link's name
            mj_pos = robot.get_body_full_mat(links[i])[:3, 3] - x0  # Get each link's relative position
            ikpy_pos = cart[i + 1][:3, 3] - x0
            print(f'mj:   {mj_pos}')
            print(f'ikpy: {ikpy_pos}')
            print(f'diff: {mj_pos - ikpy_pos}')

    # New inverse kinematics -V3
    goal_vel = np.zeros(robot.n_dof)
    max_time = 6

    x_des = [0.2, 0.88, 1.732]
    plot_joints(chain, x_des)
    q = chain.active_from_full(chain.inverse_kinematics(x_des))
    print(f'Tactile force:   {test_sensors()}')
    robot.set_mocap_pos('endeff_des', x_des)
    robot.start_spinning()
    robot.goto_joint_cubic(q, goal_vel, max_time)
    # ...and wait until it is done.
    robot.wait_for_task()
    time.sleep(1)

    x_des2 = [0.2, 0.85, 1.132]
    max_time = 6
    robot.set_mocap_pos('endeff_des', x_des2)
    plot_joints(chain, x_des2)
    q = chain.active_from_full(chain.inverse_kinematics(x_des2))
    robot.goto_joint_cubic(q, goal_vel, max_time)
    # ...and wait until it is done.
    robot.wait_for_task()
    time.sleep(1)
    # finger1.goto_joint_cubic(goal_pos_f1, goal_vel_f1, max_time)
    # finger1.wait_for_task()
    # finger1.goto_joint_cubic(goal_pos_f1, goal_vel_f1, max_time)
    # finger1.wait_for_task()
    robot.goto_home()
    robot.wait_for_task()
    robot.stop_spinning()
    robot.close()

    # robot.step(des_pos=q, n_steps=5000)


if __name__ == '__main__':
    main()
