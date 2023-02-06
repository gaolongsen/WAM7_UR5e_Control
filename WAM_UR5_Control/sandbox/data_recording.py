"""
    A test for
        - mujoco_robots.robots.MjRobot.start_recording()
        - mujoco_robots.robots.MjRobot.stop_recording()
        - mujoco_robots.robots.MjRobot.get_recording()
        - mujoco_robots.robots.MjRobot.clear_recording()

    Expected output:
        - first a 31x100 dataframe
        - then an empty one.
"""

from mujoco_robots.robots import MjWam7


def main():
    robot = MjWam7(render=False)
    robot.start_data_recording()
    robot.step(n_steps=100)
    robot.stop_data_recording()

    df = robot.get_data_recording()
    print(df.head)

    robot.clear_data_recording()
    df = robot.get_data_recording()
    print(df.head)

    # example_string = df.to_string()
    # output_file = open('file.txt','a')
    # output_file.write(example_string)
    # output_file.close()


if __name__ == '__main__':
    main()
