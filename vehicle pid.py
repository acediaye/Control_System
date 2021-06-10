import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import pid

TIME_STEP = 1
TIME_END = 300
TIME = np.arange(TIME_STEP, TIME_END+TIME_STEP, TIME_STEP)
REF = np.append(25*np.ones(int(len(TIME)/5)),
                0*np.ones(int(len(TIME)/5)))
REF = np.append(REF, 15*np.ones(int(len(TIME)/5)))
REF = np.append(REF, 20*np.ones(int(len(TIME)/5)))
REF = np.append(REF, 10*np.ones(int(len(TIME)/5)))
print(len(TIME), len(REF))


def vehicle(v, t, u):
    # inputs
    #  v    = vehicle velocity (m/s)
    #  t    = time (sec)
    #  u    = gas pedal position (-50% to 100%)
    #  load = passenger load + cargo (kg)
    Cd = 0.24  # drag coefficient
    rho = 1.225  # air density (kg/m^3)
    A = 5.0  # cross-sectional area (m^2)
    Fp = 30  # thrust parameter (N/%pedal)
    mass = 500  # vehicle mass (kg)
    load = 200  # kg
    dv_dt = (Fp*u - 0.5*rho*A*Cd*v**2)/(mass+load)
    return dv_dt


t_arr = np.array([0])  # t
v_arr = np.array([0])  # y
r_arr = np.array([0])  # r
step_arr = np.array([0])  # u
e_arr = np.array([0])  # e

KP = 1/1.2 * 5
KI = KP/30
KD = 0
prev_v = 0.0
prev_time = 0

if __name__ == '__main__':
    mypid = pid.PID(KP, KI, KD)
    # clip inputs to -50% to 100%
    mypid.set_output_saturation(-50, 100)
    mypid.set_integral_saturation(-20, 20)
    for i in range(len(TIME)):
        t = TIME[i]
        r = REF[i]
        u = mypid.controller(r, prev_v, t)
        v = odeint(vehicle, prev_v, [prev_time, t], args=(u,))
        # print(np.shape(v))
        prev_v = v[-1]   # take the last value
        t_arr = np.append(t_arr, t)  # t
        v_arr = np.append(v_arr, prev_v)  # y
        r_arr = np.append(r_arr, r)  # r
        step_arr = np.append(step_arr, u)  # u
        e_arr = np.append(e_arr, mypid.get_error())  # e
        prev_time = t

    print(mypid.count)
    print(len(t_arr))
    plt.subplot(3, 1, 1)
    plt.plot(t_arr, v_arr, 'b')  # y
    plt.plot(t_arr, r_arr, 'k')  # r
    plt.ylabel('Velocity (m/s)')
    plt.legend(['Velocity', 'Set Point'])
    plt.subplot(3, 1, 2)
    plt.plot(t_arr, step_arr, 'g')  # u
    plt.ylabel('Gas Pedal')
    plt.legend(['Gas Pedal (%)'])
    plt.xlabel('Time (sec)')
    plt.subplot(3, 1, 3)
    plt.plot(t_arr, e_arr, 'r')  # e
    plt.ylabel('Error')
    plt.xlabel('Time (sec)')
    plt.legend(['Error'])
    # mypid.graph_pid_errors()
    plt.show()
