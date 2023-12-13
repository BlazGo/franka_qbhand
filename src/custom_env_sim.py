from typing import Any, SupportsFloat
import gymnasium as gym
import tf
import rospy
import numpy as np
import os
base_dir = os.path.dirname(os.path.dirname(__file__))


def reward_function(point1: np.ndarray, point2: np.ndarray, threshold: float):
    # Calculate the Euclidean distance between the two points
    distance = np.linalg.norm(point1 - point2)
    # Check if the distance is within the threshold
    if distance <= threshold:
        # Calculate the reward as the absolute distance if inside
        reward = abs(distance)
    else:
        # Set the reward to zero if outside
        reward = 0
    return reward


def reward_function_sim(env_state: list):
    # Load the numpy file
    reward_array = np.load('reward_table_2023-10-19_9-9-7.npy')
    # Get the reward value for the given state
    reward = reward_array[env_state[0], env_state[1], env_state[2]]
    return reward


class CustomSim(gym.Env):
    """Custom Environment that follows gym interface"""

    # Environment parameters
    metadata = {'render.modes': ['ansi']}

    # dictionary returns array to be applied to state in step
    _action_to_direction = {0: np.array([1,  0,  0]),  # forward
                            1: np.array([-1, 0,  0]),  # back
                            2: np.array([0,  1,  0]),  # right
                            3: np.array([0, -1,  0]),  # left
                            4: np.array([0,  0,  1]),  # rot+
                            5: np.array([0,  0, -1])}  # rot-
    N_DISCRETE_ACTIONS = 6

    def __init__(self, observation_space_shape=None, x_range=None, y_range=None, r_range=None, ) -> None:
        super().__init__()

        r_range = r_range or [0, -60]
        y_range = y_range or [0.05, -0.05]
        x_range = x_range or [0.05, -0.05]
        observation_space_shape = observation_space_shape or [7, 7, 5]

        self.reward_table = np.load(os.path.join(base_dir, "reward_table_10_9.npy"))
        self.OBSERVATION_SPACE_SHAPE = observation_space_shape
        self.state = np.asarray(np.array(self.OBSERVATION_SPACE_SHAPE)/2, dtype=np.uint8)

        # actual state space in [m] [max, min]
        self.x_range = x_range
        self.y_range = y_range
        self.r_range = r_range

        # discrete action space definition 1x6
        self.action_space = gym.spaces.Discrete(self.N_DISCRETE_ACTIONS)

        observation_space_type = "MultiDiscrete"
        # If we're using Bayesian optimisation it may be useful to use Box observation space
        if observation_space_type == "MultiDiscrete":
            # discrete observation state space definition 7x7x5
            self.observation_space = gym.spaces.MultiDiscrete(self.OBSERVATION_SPACE_SHAPE)
        elif observation_space_type == "Box":
            self.observation_space = gym.spaces.Box(low=np.array([-0.05, -0.05, -80]),
                                                    high=np.array([0.05, 0.05, 80]),
                                                    dtype=np.float64)
        self.BASE_COORD_MODEL = [0.65, 0.0, 0.05, 0, 0, 0]
        self.BASE_COORD_ROBOT = [0.65, 0.0, 0.15, np.pi, 0, 0]

        # pack in a dictionary
        self.state_space_values = {"x": np.linspace(*x_range, num=observation_space_shape[0]),
                                   "y": np.linspace(*y_range, num=observation_space_shape[1]),
                                   "r": np.linspace(*r_range, num=observation_space_shape[2])}

    def step(self, action: Any) -> 'tuple[Any, SupportsFloat, bool, bool, dict[str, Any]]':

        _info = {}
        done = False
        terminated = False
        reward = 0

        # get array from action
        direction = self._action_to_direction[action]

        # add it to the state and clip to space
        self.state = np.clip(a=self.state + direction,
                             a_min=[0, 0, 0],
                             a_max=np.array(self.OBSERVATION_SPACE_SHAPE) - 1)

        # construct the actual robot state
        robot_state = [self.state_space_values["x"][self.state[0]],
                       self.state_space_values["y"][self.state[1]],
                       self.state_space_values["r"][self.state[2]]]

        goal_2 = [self.BASE_COORD_ROBOT[0] + robot_state[0],
                  self.BASE_COORD_ROBOT[1] + robot_state[1],
                  self.BASE_COORD_ROBOT[2] - 0.065,
                  np.pi,
                  0,
                  np.deg2rad(robot_state[2])]

        # reward
        reward = self.reward_table[self.state]

        observation = self.state
        _info["action"] = action
        _info["state"] = self.state
        _info["robot_state"] = robot_state

        return observation, reward, terminated, done, _info

    def reset(self, *, seed: int = None, options: dict = None) -> 'tuple[Any, dict[str, Any]]':
        _info = {}
        if options is None:
            self.state = np.array(self.OBSERVATION_SPACE_SHAPE) // 2
        else:
            self.state = options["state"]
        return self.state, _info


if __name__ == "__main__":
    sim = CustomSim(observation_space_shape=[7, 7, 5], x_range=[0.05, -0.05], y_range=[0.05, -0.05], r_range=[0, -60])
    state, info = sim.reset()
    print(state, info)
