import numpy as np
import matplotlib.pyplot as plt


class PID(object):
    def __init__(self, KP: float, KI: float, KD: float):
        """[PID controller]

        Parameters
        ----------
        KP : [float]
            [proportional constant]
        KI : [float]
            [integral constant]
        KD : [float]
            [derivative constant]
        """
        self.kp = KP
        self.ki = KI
        self.kd = KD
        self.error = 0
        self.prev_error = 0
        self.prev_time = 0
        self.proportional_error = 0
        self.integral_error = 0
        self.derivative_error = 0
        self.u_output = 0
        self.count = 0
        self.max_output = 100
        self.min_output = 0
        self.max_integral = 0
        self.min_integral = 0
        self.t_arr = np.array([])
        self.kpe = np.array([])
        self.kie = np.array([])
        self.kde = np.array([])

    def controller(self, reference: float, measured_value: float,
                   time: float) -> float:
        """[discrete PID calculations]

        Parameters
        ----------
        reference : float
            [setpoint]
        measured_value : float
            [feedback]
        time : float
            [time instance]

        Returns
        -------
        float
            [controller output]
        """
        time_step = time - self.prev_time
        self.error = reference - measured_value
        self.proportional_error = self.error
        self.integral_error += self.error * time_step
        # capping integral
        if self.ki * self.integral_error > self.max_integral:
            self.integral_error = self.max_output
        elif self.ki * self.integral_error < self.min_integral:
            self.integral_error = self.min_integral
        self.derivative_error = (self.error - self.prev_error) / time_step
        self.u_output = (self.kp * self.proportional_error
                         + self.ki * self.integral_error
                         + self.kd * self.derivative_error)
        self.prev_error = self.error
        self.prev_time = time
        # output saturation
        if self.u_output > self.max_output:
            self.u_output = self.max_output
        elif self.u_output < self.min_output:
            self.u_output = self.min_output
        print(f'u: {self.u_output}, r: {reference}, '
              f'y: {measured_value}, e: {self.error}')
        self.count += 1
        self.t_arr = np.append(self.t_arr, time)
        self.kpe = np.append(self.kpe, self.kp * self.proportional_error)
        self.kie = np.append(self.kie, self.ki * self.integral_error)
        self.kde = np.append(self.kde, self.kd * self.derivative_error)
        return self.u_output

    def graph_pid_errors(self):
        plt.figure()
        plt.subplot(3, 1, 1)
        plt.plot(self.t_arr, self.kpe, 'm')
        plt.ylabel('KP errors')
        plt.subplot(3, 1, 2)
        plt.plot(self.t_arr, self.kie, 'c')
        plt.ylabel('KI errors')
        plt.subplot(3, 1, 3)
        plt.plot(self.t_arr, self.kde, 'y')
        plt.ylabel('KD errors')
        plt.xlabel('Time')
        # plt.show()

    def get_error(self):
        return self.error

    def get_kpe(self):
        return self.kp * self.proportional_error

    def get_kie(self):
        return self.ki * self.integral_error

    def get_kde(self):
        return self.kd * self.derivative_error

    def ziegler_tune(self, ku: float, tu: float):
        self.kp = 0.6 * ku
        self.ki = 1.2 * ku / tu
        self.kd = 0.075 * ku * tu

    def set_output_saturation(self, min, max):
        self.min_output = min
        self.max_output = max

    def set_integral_saturation(self, min, max):
        self.min_integral = min
        self.max_integral = max
