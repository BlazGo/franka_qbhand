import rospy
import time
import itertools
import numpy as np
from rich.progress import track

from custom_env import CustomEnv
import my_log


if __name__ == "__main__":
    log = my_log.logger()
    
    rospy.init_node("gym_env")
    
    obs_shape = [9, 9, 7]
    env = CustomEnv(observation_space_shape=obs_shape)
    observation, info = env.reset()

    # --- Reward table and all states iterator ---- #
    reward_table = np.zeros(tuple(env.OBSERVATION_SPACE_SHAPE))
    
    x = range(obs_shape[0])
    y = range(obs_shape[1])
    r = range(obs_shape[2])

    # all possible combinations of lists
    states = list(itertools.product(x, y, r))
    n_states = len(states)
    
    # ----------------- Main loop ----------------- #
    i = 0
    for state in track(states, "Progress"):
        t1 = time.time()
        i += 1

        # -------- Actual step and reward --------- #
        observation, reward, terminated, done, info = env.step(action=env.action_space.sample(), abs_state=state)
        reward_table[state[0], state[1], state[2]] = reward
        
        # ------------- Save progress ------------- #
        np.save("reward_table", reward_table)
        np.save("state_space", env.state_space_values)
        
        # ------ loop time * remaining states ------ #
        t_remaining = (time.time()-t1)*(n_states -i)
        log.info(f"State: {info['state']}, Actual: {info['robot_state']}, {(i/n_states)*100:.1f}% ({i}/{n_states}) Remaining: {t_remaining//60:.0f}:{t_remaining%60:.0f}")
                