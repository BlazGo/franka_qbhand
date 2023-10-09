import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, Slider
from mpl_toolkits.mplot3d import axes3d, Axes3D
from matplotlib import cm

# TODO smooth graph, slider for r axis or some other way to visualize it

if __name__ == "__main__":
    reward_table = np.load("reward_table.npy")
    # weird unpacking saved as dict of arrays saved as array of dicts of arrays?
    state_space = np.load("state_space.npy", allow_pickle=True)[()]
        
    plt.rcParams["figure.figsize"] = [7.50, 3.50]
    
    OBSERVATION_SPACE_SHAPE = np.shape(reward_table)
    
    x = state_space["x"]
    y = state_space["y"]
    r = state_space["r"]

    print(f"Reward_table_shape = {OBSERVATION_SPACE_SHAPE}")
    print(f"State_space info = {state_space}")
    
    # ------------ PLotting ---------------- #
    xv, yv = np.meshgrid(x, y)

    fig = plt.figure()
    ax = Axes3D(fig)

    reward_layer = reward_table[:,:,5]
    scplot = ax.plot_surface(xv, yv, reward_layer, cmap="YlGnBu")
    # scplot = ax.scatter(xv, yv, r[2], c=reward_layer.flatten())
    plt.colorbar(scplot)

    ax.set_xlabel("x pos")
    ax.set_ylabel("y pos")
    ax.set_zlabel("z rot")
    
    plt.show()