from control import tf, feedback, step_response
import numpy as np
import matplotlib.pyplot as plt
import marker

# model
# input V, volts
# output theta dot, angular velocity
# T = K*i
# emf = K*theta_dot
# ------------------
# J*theta_dotdot + b*theta_dot = T
# L*di/dt + R*i = V - emf
# ----------------------
# J*s^2*theta + b*s*theta = K*I
# L*s*I + R*I = V - K*s*theta
# [s*theta(J*s + b)/k]*(L*s+R) = V-K*s*theta
# s*theta*[(J*s+b)*(L*s+R) + k^2] = k*V
# s*theta/V = k / [(J*s+b)(L*s+R)+k^2]

J = 0.01  # motor inertia kg*m^2
b = 0.1  # friction constant N*m*s
Ke = 0.01  # emf constant V/rad/s
Kt = 0.01  # motor torque constant N*m/Amp
R = 1  # resistnace ohm
L = 0.5  # inductance H

TIME_STEP = 0.1
END_TIME = 100
TIME = np.arange(TIME_STEP, END_TIME+TIME_STEP, TIME_STEP)
print(len(TIME))

KP = 100
KI = 200
KD = 10
REF = np.deg2rad(60)
s = tf('s')
plant = Kt / ((J*s+b)*(L*s+R) + Ke*Kt)
print(plant)
controller = KP + KI/s + KD*s
h = feedback(controller*plant, 1)
t, y = step_response(REF*h)
r = REF * np.ones(len(t))
plt.figure(1)
plt.plot(t, y)
plt.plot(t, r)

# scale = 100
# mark = marker.Marker(0, REF*scale)
# for i in range(len(y)):
#     pos = y[i]
#     mark.set_pos(pos*scale)

plt.show()
