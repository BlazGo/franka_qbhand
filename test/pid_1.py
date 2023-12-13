import numpy as np
from scipy.integrate import solve_ivp


class System:

    g: float = 9.81

    def __init__(self, m: float = 1, k: float = 10, c: float = 1):
        self.m = m
        self.k = k
        self.c = c

        self.x0 = 0
        self.xd0 = 0

    def spring_mass_ODE(self, t, y):
        return y[1], self.g - self.k*y[0]/self.m


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    s = System()
    sol = solve_ivp(s.spring_mass_ODE, [0, 10], (s.x0, s.xd0),
                    t_eval=np.linspace(0, 10, 5*30))

    x, x_d = sol.y
    t = sol.t

    plt.plot(t, x)
    plt.plot(t, x_d)
    plt.grid()
    plt.show()

    s_new = System()
    y = [0, 0]
    for i in range(100):
        input()
        y = s_new.spring_mass_ODE(0, y)
        print(y)