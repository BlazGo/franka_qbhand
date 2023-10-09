import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, Slider
from mpl_toolkits.mplot3d import axes3d, Axes3D
from matplotlib import cm

# TODO smooth graph, slider for r axis or some other way to visualize it

if __name__ == "__main__":
    reward_table = np.load("reward_table.npy")
    
    plt.rcParams["figure.figsize"] = [7.50, 3.50]
    #plt.rcParams["figure.autolayout"] = True    
    
    OBSERVATION_SPACE_SHAPE = [9, 9, 7]
    
    x_range=[0.05, -0.05]
    y_range=[0.05, -0.05]
    r_range=[0, -60]
    
    x = np.linspace(x_range[0], x_range[1], OBSERVATION_SPACE_SHAPE[0])
    y = np.linspace(y_range[0], y_range[1], OBSERVATION_SPACE_SHAPE[1])
    r = np.linspace(r_range[0], r_range[1], OBSERVATION_SPACE_SHAPE[2])

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