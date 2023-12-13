import matplotlib.pyplot as plt
plt.ion()


class Particle:
    gravity = 9.81
    t_step = 0.015

    def __init__(self, mass=1, spring=25, damper=1):
        self.mass = mass
        self.spring = spring
        self.damper = damper

        self.spring_pos = 0.0

        self.y = 0
        self.dy = 0
        self.ddy = 0

    def update(self, f_ext: float = 0):
        fg = self.mass * self.gravity
        fs = -self.spring * (self.y - self.spring_pos)
        fd = self.damper * self.dy
        f = fg + fs - fd + f_ext
        self.ddy = f / self.mass
        self.dy = self.dy + self.ddy * self.t_step
        self.y = self.y + self.dy * self.t_step
        return self.y


class ControllerPID:
    t_step = 0.015

    def __init__(self, kp=1, ki=0.1, kd=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.error = 0
        self.error_last = 0
        self.int_error = 0
        self.der_error = 0

        self.output = 0

    def get_control(self, ref: float = 0, real: float = 0):
        self.error = ref - real
        self.int_error += self.error * self.t_step
        self.der_error += (self.error - self.error_last) / self.t_step
        self.error_last = self.error

        self.output = self.kp*self.error + self.ki * self.int_error + self.kd*self.der_error
        return self.output


x = 0.5
t_end = 5
t_step = 0.01
n_steps = int(t_end / t_step)

particle = Particle(mass=1, spring=25, damper=5)
controller = ControllerPID(5, 25, 1.0)
y_list = []

fig, ax = plt.subplots(1, 2)
fig.canvas.draw()
renderer = fig.canvas.renderer
ax = ax.ravel()


def draw_graph_dynamic():
    ax[0].plot([x, x], [0, y], marker="o")
    ax[0].set_ylim(0, 1)
    ax[0].set_xlim(0, 1)
    ax[0].invert_yaxis()

    ax[1].plot(y_list)
    ax[1].set_ylim(0, 1)
    ax[1].set_xlim(0, n_steps)
    ax[1].invert_yaxis()

    ax[0].draw(renderer)
    ax[1].draw(renderer)
    plt.pause(t_step)
    ax[0].clear()
    ax[1].clear()


def draw_graph():
    pass


signal = 0
for step in range(n_steps):

    y = particle.update(signal)
    y_list.append(y)
    signal = controller.get_control(ref=0.5, real=y)
    print(y, signal)

    draw_graph_dynamic()
