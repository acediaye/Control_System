import numpy as np
import matplotlib.pyplot as plt
import turtle
import time

# global params
TIME_STEP = 0.1  # sec
END_TIME = 100  # sec
SETPOINT = 100  # r
INITIAL_X = 0
INITIAL_Y = 0
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
        self.screen.title('rocket')
        self.marker = Marker()
        # time.sleep(2)
        self.rocket = Rocket()
        self.pid = PID(KP, KI, KD, SETPOINT)
        self.sim = True
        self.time = 0
        self.times = np.array([])  # t
        self.height = np.array([])  # y
        self.thrust_arr = np.array([])  # u
        self.error_arr = np.array([])  # e
        self.kpe = np.array([])
        self.kie = np.array([])
        self.kde = np.array([])
        self.count = 0
        self.loop()

    def loop(self):
        while(self.sim):
            # get thrust, u output from PID
            thrust = self.pid.controller(self.rocket.get_pos())
            self.rocket.set_acc(thrust)
            self.rocket.set_vel()
            self.rocket.set_pos()
            self.time += TIME_STEP
            self.count += 1
            if self.time > END_TIME/2:
                new_target = 0
                self.marker.set_target(new_target)
                self.pid.set_target(new_target)
            if self.time > END_TIME:
                print('sim end')
                self.sim = False
            # out of bounds
            elif self.rocket.get_pos() > 1000:
                print('sim end')
                self.sim = False
            elif self.rocket.get_pos() < -1000:
                print('sim end')
                self.sim = False
            self.times = np.append(self.times, self.time)
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
        plt.ylabel('height')  # y
        plt.subplot(3, 2, 2)
        plt.plot(t, u)
        plt.ylabel('thrust')  # u
        plt.subplot(3, 2, 3)
        plt.plot(t, e)
        plt.ylabel('error')  # e

        plt.subplot(3, 2, 4)
        plt.plot(t, kpe)
        plt.ylabel('kpe')
        plt.subplot(3, 2, 5)
        plt.plot(t, kie)
        plt.ylabel('kie')
        plt.xlabel('seconds')
        plt.subplot(3, 2, 6)
        plt.plot(t, kde)
        plt.ylabel('kde')
        plt.xlabel('seconds')
        plt.show()


class Marker(object):
    def __init__(self):
        self.marker = turtle.Turtle()
        self.marker.speed(0)
        self.marker.goto(-100, 0)
        self.marker.forward(200)

        self.marker.shape('arrow')
        self.marker.color('red')
        self.marker.penup()
        self.marker.speed(0)
        self.marker.left(180)
        self.marker.goto(20, SETPOINT)

    def set_target(self, position):
        self.marker.goto(20, position)


class Rocket(object):
    # plant
    def __init__(self):
        self.rocket = turtle.Turtle()
        self.rocket.shape('turtle')
        self.rocket.color('black')
        self.rocket.penup()
        self.rocket.speed(0)
        self.rocket.left(90)
        self.rocket.goto(INITIAL_X, INITIAL_Y)
        # physics
        self.acc = INITIAL_A  # vertical accel
        self.vel = INITIAL_V
        self.pos = INITIAL_Y

    def set_acc(self, thrust):
        self.acc = g + thrust / MASS  # m/s^2 = N / kg
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
        self.setpoint = target  # r
        self.error = 0
        self.prev_error = 0
        self.proportional_error = 0
        self.integral_error = 0
        self.derivative_error = 0
        self.output = 0

    def controller(self, position):
        # position is signal y
        self.error = self.setpoint - position  # e = r - y
        self.proportional_error = self.error
        self.integral_error += self.error * TIME_STEP
        self.derivative_error = (self.error - self.prev_error) / TIME_STEP
        self.output = (self.kp * self.proportional_error
                       + self.ki * self.integral_error
                       + self.kd * self.derivative_error)  # y
        self.prev_error = self.error
        # saturation
        if self.output > MAX_THRUST:
            self.output = MAX_THRUST
        elif self.output < 0:
            self.output = 0
        print(f'u: {self.output}, r: {self.setpoint}, '
              f'y: {position}, e: {self.error}')
        return self.output  # output is signal u

    def get_err(self):
        return self.error

    def get_kpe(self):
        return self.kp * self.proportional_error

    def get_kie(self):
        return self.ki * self.integral_error

    def get_kde(self):
        return self.kd * self.derivative_error

    def set_target(self, target):
        self.setpoint = target


if __name__ == '__main__':
    sim = Simulation()
