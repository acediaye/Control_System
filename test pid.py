import pid
import numpy as np
import matplotlib.pyplot as plt

# global params
TIME_STEP = 0.1  # sec
END_TIME = 100  # sec
TIME = np.arange(0+TIME_STEP, END_TIME+TIME_STEP, TIME_STEP)
REFERENCE = 100 * np.ones(len(TIME))
MASS = 1  # kg
g = -9.8  # gravity


class model(object):
    def __init__(self, g, mass):
        self.pos = 0
        self.vel = 0
        self.acc = 0
        self.output = 0
        self.mass = mass
        self.g = g

    def excite(self, thrust):
        self.acc = self.g + thrust / self.mass
        self.vel += self.acc * TIME_STEP
        self.pos += self.vel * TIME_STEP
        return self.pos

    def get_pos(self):
        return self.pos


time_arr = np.array([])
u_arr = np.array([])
y_arr = np.array([])
r_arr = np.array([])
e_arr = np.array([])

KP = 0.6
KI = 0.06666
KD = 1.35


if __name__ == '__main__':
    mypid = pid.PID(KP, KI, KD)
    mypid.set_output_saturation(0, 20)
    mypid.set_integral_saturation(-20, 30)
    mymodel = model(g, MASS)
    for i in range(len(TIME)):
        t = TIME[i]
        r = REFERENCE[i]
        # a = 1
        # f = 0.1
        # r = a * np.sin(2*np.pi*f*t)
        u = mypid.controller(r, mymodel.get_pos(), t)
        y = mymodel.excite(u)
        time_arr = np.append(time_arr, t)
        u_arr = np.append(u_arr, u)
        y_arr = np.append(y_arr, y)
        r_arr = np.append(r_arr, r)
        e_arr = np.append(e_arr, r-y)
    print(mypid.count)
    plt.figure(1)
    plt.subplot(3, 1, 1)
    plt.plot(time_arr, y_arr, 'b')
    plt.plot(time_arr, r_arr, 'k')
    plt.ylabel('y')
    plt.subplot(3, 1, 2)
    plt.plot(time_arr, u_arr, 'g')
    plt.ylabel('u')
    plt.subplot(3, 1, 3)
    plt.plot(time_arr, e_arr, 'r')
    plt.ylabel('e')
    plt.xlabel('time')
    # mypid.graph_pid_errors()
    plt.show()
