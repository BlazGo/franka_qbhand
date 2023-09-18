import rospy
import gym
from gym import spaces

import numpy as np
import matplotlib.pyplot as plt

from gazeebo_model import Model
from panda_robot import PandaRobot


class CustomEnv(gym.Env):
    """Custom Environment that follows gym interface"""

    # Environment parameters
    metadata = {'render.modes': ['ansi']}

    # dictionary returns array to be applied to state in step
    _action_to_direction = {0: np.array([1,  0,  0]),
                            1: np.array([-1, 0,  0]),
                            2: np.array([0,  1,  0]),
                            3: np.array([0, -1,  0]),
                            4: np.array([0,  0,  1]),
                            5: np.array([0,  0, -1])}

    N_DISCRETE_ACTIONS = 6
    OBSERVATION_SPACE_SHAPE = [7, 7, 5]

    def __init__(self, observation_space_type="MultiDiscrete"):
        super(CustomEnv, self).__init__()

        self.model = Model()
        self.robot = PandaRobot()
        self.BASE_COORD_MODEL = [0.65, 0.0, 0.05, 0, 0, 0]
        self.BASE_COORD_ROBOT = [0.65, 0.0, 0.1, np.pi, 0, 0]

        # discrete action space definition 1x6
        self.action_space = gym.spaces.Discrete(self.N_DISCRETE_ACTIONS)

        # If we're using Bayesian optimisation it may be usefull to use Box observation space
        if observation_space_type == "MultiDiscrete":
            # discrete observation state space definition 7x7x5
            self.observation_space = gym.spaces.MultiDiscrete(self.OBSERVATION_SPACE_SHAPE)
        elif observation_space_type == "Box":
            self.observation_space = gym.spaces.Box(low=[-0.05, -0.05, -80],
                                                    high=[0.05, 0.05, 80],
                                                    dtype=float)

        # actual state space in [m] (symmetric)
        x_range = 0.04
        y_range = 0.04
        r_range = 30

        # pack in a dictionary
        self.state_space_values = {"x": np.linspace(-x_range, x_range, self.OBSERVATION_SPACE_SHAPE[0]),
                                   "y": np.linspace(-y_range, y_range, self.OBSERVATION_SPACE_SHAPE[1]),
                                   "r": np.linspace(-r_range, r_range, self.OBSERVATION_SPACE_SHAPE[2])}

        # move robot to inital position
        self.robot.grasp_without_force(width=0.08, t_move=2, speed=0.2)
        self.robot.cart_move_smooth(goal=self.BASE_COORD_ROBOT)

    def step(self, action: int):
        info = {}
        done = False
        reward = 0

        # get array from action
        direction = self._action_to_direction[action]

        # add it to the state and clip to space
        self.state = np.clip(a=(self.state + direction),
                             a_min=[0, 0, 0],
                             a_max=np.array(self.OBSERVATION_SPACE_SHAPE) - 1)

        # construct the actual robot state
        robot_state = [self.state_space_values["x"][self.state[0]],
                       self.state_space_values["y"][self.state[1]],
                       self.state_space_values["r"][self.state[2]]]

        # spawn model at default coordinates
        self.model.spawn(pose=self.BASE_COORD_MODEL)

        # robot action [x, y, z, roll, pitch, yaw]
        goal_1 = [self.BASE_COORD_ROBOT[0] + robot_state[0],
                  self.BASE_COORD_ROBOT[1] + robot_state[1],
                  self.BASE_COORD_ROBOT[2],
                  np.pi,
                  0,
                  np.deg2rad(robot_state[2])]
        self.robot.cart_move_smooth(goal=goal_1, t_move=2)

        goal_2 = [self.BASE_COORD_ROBOT[0] + robot_state[0],
                  self.BASE_COORD_ROBOT[1] + robot_state[1],
                  self.BASE_COORD_ROBOT[2] - 0.082,
                  np.pi,
                  0,
                  np.deg2rad(robot_state[2])]
        self.robot.cart_move_smooth(goal=goal_2, t_move=2.5)
        self.robot.grasp(width=0.035, force=4, t_move=2.0, speed=0.075)
        self.robot.cart_move_smooth(goal=goal_1, t_move=1.5)

        rospy.sleep(1.0)

        # reward
        _pose = self.model.get_state()
        if _pose.pose.position.z > 0.021:
            reward = 1
            print(_pose.pose.position)

        # reset
        self.robot.grasp_without_force(width=0.08, t_move=2)
        self.model.delete()

        observation = self.state
        info = {"action": action,
                "robot_state": robot_state}

        return observation, reward, done, info

    def reset(self, state=[0, 0, 0]):
        self.state = state
        # observation (reward, done, info musn't be included)
        return self.state

    def render(self, mode='ansi'):
        return "[INFO] "


def step(state):
    i, j, k = state
    info = {}
    done = False
    reward = 0

    z_up = 0.1
    z_grab = 0.018

    # Action
    robot.grasp_without_force(width=0.08, t_move=2, speed=0.2)
    model.spawn(pose=model_pose)

    goal = [x_base + x[i], y[j], z_up, np.pi, 0, np.deg2rad(r[k])]
    robot.cart_move_smooth(goal=goal, t_move=2)
    goal = [x_base + x[i], y[j], z_grab, np.pi, 0, np.deg2rad(r[k])]
    robot.cart_move_smooth(goal=goal, t_move=2.5)

    robot.grasp(width=0.03, force=5, t_move=2.0, speed=0.1)
    rospy.sleep(0.5)

    goal[2] = 0.1
    robot.cart_move_smooth(goal=goal, t_move=1.5)

    # Reward
    _pose = model.get_state()
    if _pose.pose.position.z > 0.021:
        reward = 1
        success_table[i, j, k] = 1
        print("[INFO] Success! Picked up hca.")
        print(_pose.pose. position)
    model.delete()

    observation = 0
    return observation, reward, done, False, info


if __name__ == "__main__":
    pass
