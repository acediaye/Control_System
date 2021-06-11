import numpy as np
import matplotlib.pyplot as plt
import turtle
import time

# global params
TIME_STEP = 0.1  # sec
END_TIME = 100  # sec
SETPOINT = 100
TIME = np.arange(0+TIME_STEP, END_TIME+TIME_STEP, TIME_STEP)
print(len(TIME))
REFERENCE = np.append(SETPOINT * np.ones(int(len(TIME)/2)),
                      np.zeros(int(len(TIME)/2)))
INITIAL_X = 0
INITIAL_Y = 0
INITIAL_V = 0  # initial velocity
INITIAL_A = 0
MASS = 1  # kg
MAX_THRUST = 20  # newtons
MAX_WINDUP = 20
g = -9.8  # gravity

# PID ziegler nichols method
KU = 1  # consistent oscillations
TU = 17  # seconds, oscillation period  # 25.5 - 8.5
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
        self.rocket = Rocket(g, MASS)
        self.pid = PID(KP, KI, KD)
        self.times = np.array([])  # t
        self.r_arr = np.array([])  # r
        self.height = np.array([])  # y
        self.thrust_arr = np.array([])  # u
        self.err_arr = np.array([])  # e
        self.kpe = np.array([])
        self.kie = np.array([])
        self.kde = np.array([])
        self.count = 0
        self.loop()

    def loop(self):
        for i in range(len(TIME)):
            t = TIME[i]
            r = REFERENCE[i]
            self.marker.set_reference(r)
            thrust = self.pid.controller(r, self.rocket.get_pos(), t)
            self.rocket.excite(thrust)
            self.count += 1

            self.times = np.append(self.times, t)
            self.r_arr = np.append(self.r_arr, r)
            self.height = np.append(self.height, self.rocket.get_pos())  # y
            self.err_arr = np.append(self.err_arr, self.pid.get_error())  # e
            self.kpe = np.append(self.kpe, self.pid.get_kpe())
            self.kie = np.append(self.kie, self.pid.get_kie())
            self.kde = np.append(self.kde, self.pid.get_kde())
            self.thrust_arr = np.append(self.thrust_arr, thrust)  # u
        self.graph(self.times, self.r_arr, self.height, self.thrust_arr,
                   self.err_arr, self.kpe, self.kie, self.kde)
        print(f'count: {self.count}')

    def graph(self, t, r, y, u, e, kpe, kie, kde):
        plt.figure(1)
        plt.subplot(3, 2, 1)
        plt.plot(t, y)
        plt.plot(t, r)
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

    def set_reference(self, position):
        self.marker.goto(20, position)


class Rocket(object):
    # plant
    def __init__(self, g, mass):
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
        self.g = g
        self.mass = mass

    def excite(self, thrust):
        # thrust is in newton, accel = force / mass
        self.acc = self.g + thrust / self.mass  # m/s^2 = N / kg
        print(f'accel: {self.acc}')
        self.vel += self.acc * TIME_STEP
        print(f'vel: {self.vel}')
        self.pos += self.vel * TIME_STEP
        print(f'pos: {self.pos}\n')
        self.rocket.sety(self.pos)

    def get_acc(self):
        return self.acc

    def get_vel(self):
        return self.vel

    def get_pos(self):
        self.pos = self.rocket.ycor()
        return self.pos


class PID(object):
    # controller
    def __init__(self, KP, KI, KD):
        self.kp = KP
        self.ki = KI
        self.kd = KD
        # self.setpoint = 0  # r
        self.error = 0
        self.prev_error = 0
        self.prev_time = 0
        self.proportional_error = 0
        self.integral_error = 0
        self.derivative_error = 0
        self.output = 0

    def controller(self, reference, position, time):
        # position is signal y
        self.error = reference - position  # e = r - y
        self.proportional_error = self.error
        self.integral_error += self.error * (time - self.prev_time)
        # capping integral error
        # if self.integral_error > MAX_WINDUP:
        #     self.integral_error = MAX_WINDUP
        # elif self.integral_error < -MAX_WINDUP:
        #     self.integral_error = -MAX_WINDUP
        self.derivative_error = ((self.error - self.prev_error)
                                 / (time - self.prev_time))
        self.output = (self.kp * self.proportional_error
                       + self.ki * self.integral_error
                       + self.kd * self.derivative_error)  # y
        self.prev_error = self.error
        self.prev_time = time
        # saturation
        if self.output > MAX_THRUST:
            self.output = MAX_THRUST
        elif self.output < 0:
            self.output = 0
        print(f'u: {self.output}, r: {reference}, '
              f'y: {position}, e: {self.error}')
        return self.output  # output is signal u

    def get_error(self):
        return self.error

    def get_kpe(self):
        return self.kp * self.proportional_error

    def get_kie(self):
        return self.ki * self.integral_error

    def get_kde(self):
        return self.kd * self.derivative_error


if __name__ == '__main__':
    sim = Simulation()
