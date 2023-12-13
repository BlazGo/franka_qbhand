import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import gymnasium as gym
from stable_baselines3.common.env_checker import check_env

import numpy as np

import my_log
from gazeebo_model import Model
from panda_robot import PandaRobot


class CustomEnv(gym.Env):
    """Custom Environment that follows gym interface"""

    # Environment parameters
    metadata = {'render.modes': ['ansi']}

    # dictionary returns array to be applied to state in step
    _action_to_direction = {0: np.array([1, 0, 0]),  # forward
                            1: np.array([-1, 0, 0]),  # back
                            2: np.array([0, 1, 0]),  # right
                            3: np.array([0, -1, 0]),  # left
                            4: np.array([0, 0, 1]),  # rot+
                            5: np.array([0, 0, -1])}  # rot-
    N_DISCRETE_ACTIONS = 6

    def __init__(self, observation_space_shape=[7, 7, 5],
                 x_range=[0.05, -0.05],
                 y_range=[0.05, -0.05],
                 r_range=[0, -60],
                 observation_space_type="MultiDiscrete", log_level=my_log.INFO):
        super(CustomEnv, self).__init__()

        assert len(observation_space_shape) == 3, "Observation space shape " + \
                                                  "is currently expected to be only of length 3!"
        self.OBSERVATION_SPACE_SHAPE = observation_space_shape

        # actual state space in [m] [max, min]
        self.x_range = x_range
        self.y_range = y_range
        self.r_range = r_range

        self.log = my_log.Logger(level=log_level)

        self.model = Model(log_level=my_log.WARNING)
        self.robot = PandaRobot(log_level=my_log.WARNING, move_feedback=True)
        self.BASE_COORD_MODEL = [0.65, 0.0, 0.05, 0, 0, 0]
        self.BASE_COORD_ROBOT = [0.65, 0.0, 0.15, np.pi, 0, 0]

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

        # pack in a dictionary
        self.state_space_values = {"x": np.linspace(x_range[0], x_range[1], self.OBSERVATION_SPACE_SHAPE[0]),
                                   "y": np.linspace(y_range[0], y_range[1], self.OBSERVATION_SPACE_SHAPE[1]),
                                   "r": np.linspace(r_range[0], r_range[1], self.OBSERVATION_SPACE_SHAPE[2])}
        self.state = [0, 0, 0]
        # move robot to initial position
        self.robot.grasp(value=0.0)
        self.robot.cart_move_smooth(trans=self.BASE_COORD_ROBOT[0:3],
                                    rot=quaternion_from_euler(self.BASE_COORD_ROBOT[3], self.BASE_COORD_ROBOT[4],
                                                              self.BASE_COORD_ROBOT[5]))

    def step(self, action: int, abs_state=None):
        info = {}
        done = False
        terminated = False
        reward = 0

        # get array from action
        direction = self._action_to_direction[action]

        if type(abs_state) is list:
            # add it to the state and clip to space
            self.state = np.clip(a=self.state + direction,
                                 a_min=[0, 0, 0],
                                 a_max=np.array(self.OBSERVATION_SPACE_SHAPE) - 1)
        else:
            # option to go to any state
            self.state = abs_state

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

        self.robot.cart_move_smooth(trans=goal_1[0:3],
                                    rot=quaternion_from_euler(*goal_1[3:6]),
                                    t_move=2.0)

        goal_2 = [self.BASE_COORD_ROBOT[0] + robot_state[0],
                  self.BASE_COORD_ROBOT[1] + robot_state[1],
                  self.BASE_COORD_ROBOT[2] - 0.065,
                  np.pi,
                  0,
                  np.deg2rad(robot_state[2])]

        self.robot.cart_move_smooth(trans=goal_2[0:3], rot=quaternion_from_euler(*goal_2[3:6]))
        self.robot.cart_move_smooth(trans=goal_2[0:3], rot=quaternion_from_euler(*goal_2[3:6]), t_move=0.5)

        self.robot.grasp(value=1.0, t=3.0)
        self.robot.cart_move_smooth(trans=goal_1[0:3], rot=quaternion_from_euler(*goal_1[3:6]))

        rospy.sleep(1.0)

        # reward
        _pose = self.model.get_state()
        if _pose.pose.position.z > 0.04:
            reward = 1
            self.log.info(f"@{self.model.MODEL_NAME} picked up at state {self.state}")
        # reset
        self.robot.grasp(value=0.0)
        self.model.delete()
        self.robot.cart_move_smooth(trans=self.BASE_COORD_ROBOT[0:3],
                                    rot=quaternion_from_euler(self.BASE_COORD_ROBOT[3], self.BASE_COORD_ROBOT[4],
                                                              self.BASE_COORD_ROBOT[5]))

        observation = self.state
        info = {"action": action,
                "state": self.state,
                "robot_state": robot_state}

        return observation, reward, terminated, done, info

    def reset(self, seed=None, state=np.array([0, 0, 0])):
        info = {}
        self.state = state

        # observation (reward, done, info mustn't be included)
        return self.state, info

    def render(self, mode='ansi'):
        return "[INFO] "


if __name__ == "__main__":
    import my_log

    log = my_log.Logger()

    rospy.init_node("gym_env")
    env = CustomEnv()

    # Check gym env conformity
    check_env(env)

    observation, info = env.reset()
