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

        self.log = my_log.logger(level=my_log.INFO)

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

        # actual state space in [m] (symmetric)
        x_range = 0.05
        y_range = 0.05
        r_range = 30

        # pack in a dictionary
        self.state_space_values = {"x": np.linspace(-x_range, x_range, self.OBSERVATION_SPACE_SHAPE[0]),
                                   "y": np.linspace(-y_range, y_range, self.OBSERVATION_SPACE_SHAPE[1]),
                                   "r": np.linspace(-r_range, r_range, self.OBSERVATION_SPACE_SHAPE[2])}

        # move robot to inital position
        self.robot.grasp(value=0.0)
        self.robot.cart_move_smooth(trans=self.BASE_COORD_ROBOT[0:3],
                                    rot=quaternion_from_euler(self.BASE_COORD_ROBOT[3], self.BASE_COORD_ROBOT[4], self.BASE_COORD_ROBOT[5]))

    def step(self, action: int, abs_state=None):
        info = {}
        done = False
        reward = 0

        # get array from action
        direction = self._action_to_direction[action]

        if type(abs_state) != list:
            # add it to the state and clip to space
            self.state = np.clip(a     = self.state + direction,
                                a_min = [0, 0, 0],
                                a_max = np.array(self.OBSERVATION_SPACE_SHAPE) - 1)
        else:
            # option to go to any state
            self.state = abs_state


        # construct the actual robot state
        robot_state = [self.state_space_values["x"][self.state[0]],
                       self.state_space_values["y"][self.state[1]],
                       self.state_space_values["r"][self.state[2]]]
        
        self.log.info(f"State: {state} Actual: {robot_state}")

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
                  self.BASE_COORD_ROBOT[2] - 0.065,
                  np.pi,
                  0,
                  np.deg2rad(robot_state[2])]
        self.robot.cart_move_smooth(trans=goal_2[0:3], rot=quaternion_from_euler(*goal_2[3:6]))
        self.robot.grasp(value=1.0, t=4.0)
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
    import itertools
    import my_log
    import time
    
    log = my_log.logger()
    
    rospy.init_node("gym_env")
    env = CustomEnv()
    
    reward_table = np.zeros(tuple(env.OBSERVATION_SPACE_SHAPE))
    observation, info = env.reset()
    
    x = range(env.OBSERVATION_SPACE_SHAPE[0])
    y = range(env.OBSERVATION_SPACE_SHAPE[1])
    r = range(env.OBSERVATION_SPACE_SHAPE[2])

    states = itertools.product(x, y, r)
    n_states = len(x) * len(y) * len(r)
    
    i = 0
    for state in states:
        t1 = time.time()
        i += 1

        # ------ Actual step and reward ------ #
        observation, reward, done, info = env.step(action=env.action_space.sample(), abs_state=state)
        reward_table[state[0], state[1], state[2]] = reward
        
        # save progress
        np.save("reward_table", reward_table)
        
        # loop time * remaining states
        t_remaining = (time.time()-t1)*(n_states -i)
        log.info(f"Remaining: {(i/n_states)*100:.1f}% ({i}/{n_states}) t: {t_remaining//60:.0f}:{t_remaining%60:.0f}")
                