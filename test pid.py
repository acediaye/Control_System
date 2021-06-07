import pid
import numpy as np
import matplotlib.pyplot as plt

END_TIME = 100
TIME_STEP = 1
REFERENCE = 100


class model(object):
    def __init__(self):
        self.pos = 0
        self.vel = 0
        self.acc = 0

    def func(self, input):
        # output = input*REFERENCE
        self.acc = -9.8 + input / 1
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


if __name__ == '__main__':
    mypid = pid.PID(0.6, 0.06666, 1.35, REFERENCE)
    mymodel = model()
    for t in np.arange(TIME_STEP, END_TIME+TIME_STEP, TIME_STEP):
        u = mypid.controller(mymodel.get_pos(), t)
        y = mymodel.func(u)
        time_arr = np.append(time_arr, t)
        u_arr = np.append(u_arr, u)
        y_arr = np.append(y_arr, y)
        r_arr = np.append(r_arr, REFERENCE)
        e_arr = np.append(e_arr, REFERENCE-y)
    print(mypid.count)
    plt.figure(1)
    plt.subplot(4, 1, 1)
    plt.plot(time_arr, u_arr)
    plt.ylabel('u')
    plt.subplot(4, 1, 2)
    plt.plot(time_arr, y_arr)
    plt.ylabel('y')
    plt.subplot(4, 1, 3)
    plt.plot(time_arr, r_arr)
    plt.ylabel('r')
    plt.subplot(4, 1, 4)
    plt.plot(time_arr, e_arr)
    plt.ylabel('e')
    plt.show()
