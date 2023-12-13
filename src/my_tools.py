import numpy as np
import math
  
def Rx(theta):
  return np.matrix([[ 1, 0           , 0           ],
                    [ 0, math.cos(theta),-math.sin(theta)],
                    [ 0, math.sin(theta), math.cos(theta)]])
  
def Ry(theta):
  return np.matrix([[ math.cos(theta), 0, math.sin(theta)],
                    [ 0           , 1, 0           ],
                    [-math.sin(theta), 0, math.cos(theta)]])
  
def Rz(theta):
  return np.matrix([[ math.cos(theta), -math.sin(theta), 0 ],
                    [ math.sin(theta), math.cos(theta) , 0 ],
                    [ 0           , 0            , 1 ]])
  
def get_rotation_matrix(angle:float=0, axis:str="x"):
    if axis == "x":
        return Rx(angle)
    elif axis == "y":
        return Ry(angle)
    elif axis == "z":
        return Rz(angle)

 
def euler_from_quaternion(quat):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x, y, z, w = quat
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return [roll_x, pitch_y, yaw_z] # in radians

def interpolate(t:float):
    # smooth 0-1 interval curve
    return t*t*t*(t*(t*6 - 15) + 10)
 
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import my_log
    log = my_log.Logger()
    
    angle = 45 # [°] angle
    axis = "x"
    log.info(get_rotation_matrix(np.deg2rad(angle), axis))
    
    t = np.arange(0, 1, 0.05)
    plt.plot(interpolate(t))
    plt.show()