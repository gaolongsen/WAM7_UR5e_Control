# Author: Longsen Gao
# Contact: longsengao@gmail.com

import numpy as np
from mujoco_robots.robots import MjWam7, MjWam4

if __name__ == "__main__":
    robot = MjWam7(render=True)
    # robot = MjWam4(render=True)

    goal_state = np.ones(robot.n_dof)

    def compute_des_state(start, goal, max_time, t):
        des_pos = np.minimum(1., (t / max_time)) * (goal - start) + start
        des_vel = np.zeros_like(start)
        return des_pos, des_vel

    for _ in range(2):
        pos, vel, time = robot.reset()
        for i in range(2000):
            des_pos, des_vel = compute_des_state(robot.home_pos, goal_state, 3., time)

            # works similar to openai gym but also has a pd(+g) controller, so you can
            # specify desired positions, velocities and feed forward torques.
            pos, vel, time = robot.step(des_pos, des_vel)

