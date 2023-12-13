import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, Slider
from mpl_toolkits.mplot3d import axes3d, Axes3D
from rich import print


def plot_reward(r_arr, s_dict):
    arr_shape = np.shape(r_arr)
    x = s_dict["x"]
    y = s_dict["y"]
    r = s_dict["r"]
    print(f"Reward_table_shape = ", arr_shape)
    print(f"State_space info = ", s_dict)

    # ------------ Plotting ---------------- #
    xv, yv = np.meshgrid(x, y)

    fig = plt.figure()
    ax = Axes3D(fig)

    reward_layer = np.sum(reward_table, axis=2)
    scatter_plot = ax.plot_surface(xv, yv, reward_layer, cmap="YlGnBu")
    # scatter_plot = ax.scatter(xv, yv, r[2], c=reward_layer.flatten())
    plt.colorbar(scatter_plot)

    ax.set_xlabel("x pos")
    ax.set_ylabel("y pos")
    ax.set_zlabel("reward")

    plt.show()

# TODO smooth graph, slider for r axis or some other way to visualize it


if __name__ == "__main__":
    import  os
    BASE_DIR = os.path.dirname(os.path.dirname(__file__))

    reward_table = np.load(os.path.join(BASE_DIR, "reward_table_10_9.npy"))
    # weird unpacking saved as dict of arrays saved as array of dicts of arrays?
    state_space = np.load(os.path.join(BASE_DIR, "state_space_10_9.npy"), allow_pickle=True)[()]

    plot_reward(reward_table, state_space)
