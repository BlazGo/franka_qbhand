import numpy as np
import math as m
  
def Rx(theta):
  return np.matrix([[ 1, 0           , 0           ],
                    [ 0, m.cos(theta),-m.sin(theta)],
                    [ 0, m.sin(theta), m.cos(theta)]])
  
def Ry(theta):
  return np.matrix([[ m.cos(theta), 0, m.sin(theta)],
                    [ 0           , 1, 0           ],
                    [-m.sin(theta), 0, m.cos(theta)]])
  
def Rz(theta):
  return np.matrix([[ m.cos(theta), -m.sin(theta), 0 ],
                    [ m.sin(theta), m.cos(theta) , 0 ],
                    [ 0           , 0            , 1 ]])
  
def get_rotation_matrix(angle:float=0, axis:str="x"):
    if axis == "x":
        return Rx(angle)
    elif axis == "y":
        return Ry(angle)
    elif axis == "z":
        return Rz(angle)

def interpolate(t:float):
    # smooth 0-1 interval curve
    return t*t*t*(t*(t*6 - 15) + 10)
 
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import my_log
    log = my_log.logger()
    
    angle = 45 # [°] angle
    axis = "x"
    log.info(get_rotation_matrix(np.deg2rad(angle), axis))
    
    t = np.arange(0, 1, 0.05)
    plt.plot(interpolate(t))
    plt.show()