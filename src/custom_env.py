import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import gym
import numpy as np
import matplotlib.pyplot as plt
import my_log
from gazeebo_model import Model
from panda_robot import PandaRobot


class CustomEnv(gym.Env):
    """Custom Environment that follows gym interface"""

    # Environment parameters
    metadata = {'render.modes': ['ansi']}

    # dictionary returns array to be applied to state in step
    _action_to_direction = {0: np.array([1,  0,  0]), # forward
                            1: np.array([-1, 0,  0]), # back
                            2: np.array([0,  1,  0]), # right
                            3: np.array([0, -1,  0]), # left
                            4: np.array([0,  0,  1]), # rot+
                            5: np.array([0,  0, -1])} # rot-

    N_DISCRETE_ACTIONS = 6
    OBSERVATION_SPACE_SHAPE = [7, 7, 5]

    def __init__(self, observation_space_type="MultiDiscrete", log_level=my_log.INFO):
        super(CustomEnv, self).__init__()

        log = my_log.logger(level=log_level)

        self.model = Model()
        self.robot = PandaRobot(move_feedback=True)
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

        # actual state space in [m] (symmetric)
        x_range = 0.04
        y_range = 0.04
        r_range = 30

        # pack in a dictionary
        self.state_space_values = {"x": np.linspace(-x_range, x_range, self.OBSERVATION_SPACE_SHAPE[0]),
                                   "y": np.linspace(-y_range, y_range, self.OBSERVATION_SPACE_SHAPE[1]),
                                   "r": np.linspace(-r_range, r_range, self.OBSERVATION_SPACE_SHAPE[2])}

        # move robot to inital position
        self.robot.grasp(value=0.0)
        self.robot.cart_move_smooth(trans=self.BASE_COORD_ROBOT[0:3],
                                    rot=quaternion_from_euler(self.BASE_COORD_ROBOT[3], self.BASE_COORD_ROBOT[4], self.BASE_COORD_ROBOT[5]))

    def step(self, action: int):
        info = {}
        done = False
        reward = 0

        # get array from action
        direction = self._action_to_direction[action]

        # add it to the state and clip to space
        self.state = np.clip(a     = self.state + direction,
                             a_min = [0, 0, 0],
                             a_max = np.array(self.OBSERVATION_SPACE_SHAPE) - 1)

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
        self.robot.cart_move_smooth(trans=goal_1[0:3], rot=quaternion_from_euler(*goal_1[3:6]))

        goal_2 = [self.BASE_COORD_ROBOT[0] + robot_state[0],
                  self.BASE_COORD_ROBOT[1] + robot_state[1],
                  self.BASE_COORD_ROBOT[2] - 0.1,
                  np.pi,
                  0,
                  np.deg2rad(robot_state[2])]
        self.robot.cart_move_smooth(trans=goal_2[0:3], rot=quaternion_from_euler(*goal_2[3:6]))
        self.robot.grasp(value=1.0, t=4.0)
        self.robot.cart_move_smooth(trans=goal_1[0:3], rot=quaternion_from_euler(*goal_1[3:6]))

        rospy.sleep(1.0)

        # reward
        _pose = self.model.get_state()
        if _pose.pose.position.z > 0.02:
            reward = 1
            print(_pose.pose.position)

        # reset
        self.robot.grasp(value=0.0)
        self.model.delete()
        self.robot.cart_move_smooth(trans=self.BASE_COORD_ROBOT[0:3],
                                    rot=quaternion_from_euler(self.BASE_COORD_ROBOT[3], self.BASE_COORD_ROBOT[4], self.BASE_COORD_ROBOT[5]))
        
        observation = self.state
        info = {"action": action,
                "robot_state": robot_state}

        return observation, reward, done, info

    def reset(self, state=[0, 0, 0]):
        info = {}
        self.state = state
        
        # observation (reward, done, info musn't be included)
        return self.state, info

    def render(self, mode='ansi'):
        return "[INFO] "


if __name__ == "__main__":
    rospy.init_node("gym_env")
    env = CustomEnv()
    print(env.action_space, env.observation_space)
    
    x = np.linspace(start=-0.01, stop=0.01, num= 3)
    y = np.linspace(start=-0.01, stop=0.01, num= 3)
    r = np.linspace(start=-10, stop=10, num=3)

    success_table = np.zeros((len(x), len(y), len(r)))
    observation, info = env.reset()
    
    env.step(action=env.action_space.sample())