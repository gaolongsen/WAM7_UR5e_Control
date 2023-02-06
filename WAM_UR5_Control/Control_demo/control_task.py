# Author: Longsen Gao
# Contact: longsengao@gmail.com
# Created Time： 12-14-2022

from __future__ import print_function
import time
import numpy as np
import os

from numpy import array
from mujoco_robots.robots import MjWam7_pure, UR5_pure, test_sensors
from numpy.linalg import norm, solve
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialmath.base import transl
from Pose_Get import WAM_UR5_Pose

if __name__ == "__main__":
    related_cartesian = np.array(
        [0.53056503, -0.02877194, 1.35359])  # Transfer the cartesian system from the base link to world frame
    pose = "<key qpos='-0.156 1.56 -0.392 0 0.446 1.6 0.022 0 0 0.8164 -1.5386 -1.6014 0 0 0'/>"
    WAM_pose, UR5_pose = WAM_UR5_Pose(pose)  # Get pose information from Mujoco for both WAM and UR5
    print("\n WAM_Pose:", WAM_pose)
    print("\n UR5_Pose:", UR5_pose)
    robot_wam = rtb.models.WAM()
    robot_ur5 = rtb.models.UR5()

    print(robot_wam)
    print(robot_ur5)
    Te = robot_wam.fkine(WAM_pose)      # Get SE(3) for the end-effector of WAM
    Te_u = robot_ur5.fkine(UR5_pose)    # Get SE(3) for the end-effector of UR5
    print(Te)
    print(Te_u)

    # For WAM
    pos_get = transl(Te.A)  # Get position (x, y z) from SE(3)
    rot_get = Te.R  # Get rotation matrix 3-by-3 from SE(3)

    # For UR5
    pos_ge_u = transl(Te_u.A)  # Get position (x, y z) from SE(3)
    rot_get_u = Te_u.R  # Get rotation matrix 3-by-3 from SE(3)

    # Set up the red point position
    end_pos = pos_get + related_cartesian

    Target_pos = np.array([0.60550, 0.58666, 1.53]) - related_cartesian
    # Change the end-effector position in related cartesian frame
    change_part = Target_pos - pos_get

    Te = SE3.Trans(change_part) * Te
    sol = robot_wam.ik_lm_chan(Te)  # solve IK
    end_pos = end_pos + change_part
    print(sol)

    q_pickup = sol[0]
    print(robot_wam.fkine(q_pickup))
    test = q_pickup.tolist()
    robot_wam = MjWam7_pure(render=True)
    robot_ur5 = UR5_pure(render=True)

    # goal_pos = np.ones(robot.n_dof)
    # os.system('sh /home/wam/Desktop/test1.sh')
    goal_vel = np.zeros(robot_wam.n_dof)
    goal_vel_u = np.zeros(robot_ur5.n_dof)
    max_time = 10.

    robot_wam.set_mocap_pos('endeff_des', end_pos)
    robot_wam.start_spinning()
    # robot_ur5.start_spinning()
    # define a high level task...
    # robot.goto_joint_cubic([1.571, 0.5, 0.0, 0.0, 1.9, 0.0, 0.5, 2.33, 0.84, 0.0, 2.33, 0.84, 0, 2.33, 0.84],
    # goal_vel, max_time)
    # time.sleep(1)
    # ...and wait until it is done.
    # robot.wait_for_task()
    robot_wam.goto_joint_cubic(test, goal_vel, max_time)
    # robot_ur5.goto_joint_cubic(UR5_pose, goal_vel_u, max_time)
    # robot_ur5.wait_for_task()

    robot_wam.wait_for_task()

    robot_wam.goto_home()
    robot_wam.wait_for_task()

    robot_wam.stop_spinning()
    robot_wam.close()
