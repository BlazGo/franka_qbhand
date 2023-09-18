import gym
import matplotlib.pyplot as plt
from gym import spaces

class MyEnv(gym.Env):
    def __init__(self) -> None:
        super(MyEnv, self).__init__()

        self.observation_shape = (600, 800, 3)
        self.observation_space = spaces.Box)=

if __name__ == "__main__":
    pass