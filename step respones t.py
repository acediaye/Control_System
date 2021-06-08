import pid
import numpy as np
import matplotlib.pyplot as plt

# mass spring damper system
# mx.. + bx. + kx = F
# Kp + Ki/s + Kd*s

m = 1  # kg
b = 10  # N
k = 20  # N/m
f = 1  # N

KP = 350
KI = 300
KD = 50

TIME_STEP = 0.1
END_TIME = 100
TIME = np.arange(0+TIME_STEP, END_TIME+TIME_STEP, TIME_STEP)
REFERENCE = np.ones(len(TIME))


class model(object):
    def __init__(self, m, b, k):
        self.mass = m
        self.b = b  # friction constant
        self.k = k  # spring constant
        self.acc = 0
        self.vel = 0
        self.pos = 0
        self.y = 0

    def excite(self, u):
        # mx.. + bx. + kx = F
        # output = displacement
        # input = force
        # a = F/m
        self.acc = u / self.mass
        self.vel += self.acc * TIME_STEP
        self.pos += self.vel * TIME_STEP
        self.y = (u - (self.mass * self.acc) - (self.b * self.vel)) / self.k
        return self.y

    def get_y(self):
        return self.y


t_arr = np.array([])
u_arr = np.array([])
y_arr = np.array([])
r_arr = np.array([])
e_arr = np.array([])
KP = 1
KI = 0
KD = 0
if __name__ == '__main__':
    mypid = pid.PID(KP, KI, KD)
    mymodel = model(m, b, k)
    for i in range(len(TIME)):
        t = TIME[i]
        r = REFERENCE[i]
        u = mypid.controller(r, mymodel.get_y(), t)
        y = mymodel.excite(u)
        t_arr = np.append(t_arr, t)
        u_arr = np.append(u_arr, u)
        y_arr = np.append(y_arr, y)
        r_arr = np.append(r_arr, r)
        e_arr = np.append(e_arr, r-y)
    print(mypid.count)
    plt.figure(1)
    plt.subplot(3, 1, 1)
    plt.plot(t_arr, y_arr)
    plt.plot(t_arr, r_arr)
    plt.ylabel('y')
    plt.subplot(3, 1, 2)
    plt.plot(t_arr, u_arr)
    plt.ylabel('u')
    plt.subplot(3, 1, 3)
    plt.plot(t_arr, e_arr)
    plt.ylabel('e')
    plt.show()
