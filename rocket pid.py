import numpy as np
import matplotlib.pyplot as plt
import turtle

# global params
TIME_STEP = 0.1  # sec
END_TIME = 100  # sec
SETPOINT = 10
INITIAL_X = 0
INITIAL_Y = -100
INITIAL_V = 0  # initial velocity
INITIAL_A = 0
MASS = 1  # kg
MAX_THRUST = 20  # newtons
g = -9.8  # gravity

# PID ziegler nichols method
KU = 1  # consistent oscillations
TU = 18  # seconds, oscillation period  # 27 - 9
KP = 0.6*KU
KI = 1.2*KU/TU
KD = 0.075*KU*TU
# KP = 1
# KI = 0
# KD = 0


class Simulation(object):
    def __init__(self):
        self.screen = turtle.Screen()
        self.screen.setup(600, 400)
        self.init_marker()
        self.rocket = Rocket()
        self.pid = PID(KP, KI, KD, SETPOINT)
        self.sim = True
        self.timer = 0
        self.times = np.array([])  # t
        self.height = np.array([])  # y
        self.thrust_arr = np.array([])  # u
        self.error_arr = np.array([])  # e
        self.kpe = np.array([])
        self.kie = np.array([])
        self.kde = np.array([])
        self.count = 0

    def init_marker(self):
        marker = turtle.Turtle()
        marker.penup()
        marker.speed(0)
        marker.left(180)
        marker.goto(15, SETPOINT)
        marker.color('red')

    def loop(self):
        while(self.sim):
            # get thrust, u output from our PID
            thrust = self.pid.compute(self.rocket.get_pos())
            self.rocket.set_acc(thrust)
            self.rocket.set_vel()
            self.rocket.set_pos()
            self.timer += TIME_STEP
            self.count += 1
            if self.timer > END_TIME:
                print('sim end')
                self.sim = False
            # out of bounds
            elif self.rocket.get_pos() > 800:
                print('sim end')
                self.sim = False
            elif self.rocket.get_pos() < -800:
                print('sim end')
                self.sim = False
            self.times = np.append(self.times, self.timer)
            self.height = np.append(self.height, self.rocket.get_pos())  # y
            self.error_arr = np.append(self.error_arr, self.pid.get_err())  # e
            self.kpe = np.append(self.kpe, self.pid.get_kpe())
            self.kie = np.append(self.kie, self.pid.get_kie())
            self.kde = np.append(self.kde, self.pid.get_kde())
            self.thrust_arr = np.append(self.thrust_arr, thrust)  # u
        self.graph(self.times, self.height, self.thrust_arr,
                   self.error_arr, self.kpe, self.kie, self.kde)
        print(f'count: {self.count}')

    def graph(self, t, y, u, e, kpe, kie, kde):
        plt.figure(1)
        plt.subplot(3, 2, 1)
        plt.plot(t, y)
        plt.ylabel('height')
        plt.subplot(3, 2, 2)
        plt.plot(t, u)
        plt.ylabel('thrust')
        plt.subplot(3, 2, 3)
        plt.plot(t, e)
        plt.ylabel('error')

        plt.subplot(3, 2, 4)
        plt.plot(t, kpe)
        plt.ylabel('kpe')
        plt.subplot(3, 2, 5)
        plt.plot(t, kie)
        plt.ylabel('kie')
        plt.subplot(3, 2, 6)
        plt.plot(t, kde)
        plt.ylabel('kde')
        plt.xlabel('seconds')
        plt.show()


class Rocket(object):
    # plant
    def __init__(self):
        self.rocket = turtle.Turtle()
        self.rocket.shape('square')
        self.rocket.color('black')
        self.rocket.penup()
        self.rocket.speed(0)
        self.rocket.goto(INITIAL_X, INITIAL_Y)
        # physics
        self.acc = INITIAL_A  # vertical accel
        self.vel = INITIAL_V
        self.pos = INITIAL_Y

    def set_acc(self, thrust):
        self.acc = g + thrust / MASS
        print(f'accel: {self.acc}')

    def get_acc(self):
        return self.acc

    def set_vel(self):
        self.vel += self.acc * TIME_STEP
        print(f'vel: {self.vel}')

    def get_vel(self):
        return self.vel

    def set_pos(self):
        self.pos += self.vel * TIME_STEP
        print(f'pos: {self.pos}\n')
        self.rocket.sety(self.pos)

    def get_pos(self):
        self.pos = self.rocket.ycor()
        return self.pos


class PID(object):
    # controller
    def __init__(self, KP, KI, KD, target):
        self.kp = KP
        self.ki = KI
        self.kd = KD
        self.setpoint = target
        self.error = 0
        self.prev_error = 0
        self.proportional_error = 0
        self.integral_error = 0
        self.derivative_error = 0
        self.output = 0

    def compute(self, position):
        # position is signal y
        self.error = self.setpoint - position
        self.proportional_error = self.error
        self.integral_error += self.error * TIME_STEP
        self.derivative_error = (self.error - self.prev_error) / TIME_STEP
        self.output = (self.kp * self.proportional_error
                       + self.ki * self.integral_error
                       + self.kd * self.derivative_error)
        self.prev_error = self.error
        # saturation
        if self.output > MAX_THRUST:
            self.output = MAX_THRUST
        elif self.output < 0:
            self.output = 0
        print(f'out: {self.output}')
        return self.output  # output is signal u

    def get_err(self):
        return self.error

    def get_kpe(self):
        return self.kp * self.proportional_error

    def get_kie(self):
        return self.ki * self.integral_error

    def get_kde(self):
        return self.kd * self.derivative_error


if __name__ == '__main__':
    sim = Simulation()
    sim.loop()
