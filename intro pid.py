from control import tf, feedback, series, step_response, pzmap, bode_plot
import matplotlib.pyplot as plt
import numpy as np
import marker

# model
# mass spring damper system
# input F, force
# output x, position
# m: mass, b: friction constant, k: spring constant
# mx.. + bx. + kx = F
# ms^2X + bsX + kX = F
# X/F = 1 / (ms^2 + bs + k)
# Kp + Ki/s + Kd*s

m = 1  # kg
b = 10  # N
k = 20  # N/m
f = 1  # N

KP = 350
KI = 300
KD = 50

s = tf('s')
plant = 1 / (m*s**2 + b*s + k)

# open loop step response
t, y = step_response(plant)
print(f'open: {plant}')
plt.figure(1)
plt.subplot(2, 4, 1)
plt.plot(t, y)
# reference = np.ones(len(t))
# plt.plot(t, reference)
plt.title('Open loop')
plt.xlabel('time')
plt.ylabel('amplitude')

# proportional control
controller = KP
pc = series(controller, plant)
h = feedback(pc, 1)
t, y = step_response(h)
print(f'P TF: {h}')
plt.subplot(2, 4, 2)
plt.plot(t, y)
reference = np.ones(len(t))
plt.plot(t, reference)
plt.title('Proportional')
plt.xlabel('time')
plt.ylabel('amplitude')

# integral control
controller = KI/s
pc = series(controller, plant)
h = feedback(pc, 1)
t, y = step_response(h)
print(f'I TF: {h}')
plt.subplot(2, 4, 3)
plt.plot(t, y)
reference = np.ones(len(t))
plt.plot(t, reference)
plt.title('Integral')
plt.xlabel('time')
plt.ylabel('amplitude')

# derivative control
controller = KD*s
pc = series(controller, plant)
h = feedback(pc, 1)
t, y = step_response(h)
print(f'D TF: {h}')
plt.subplot(2, 4, 4)
plt.plot(t, y)
reference = np.ones(len(t))
plt.plot(t, reference)
plt.title('Derivative')
plt.xlabel('time')
plt.ylabel('amplitude')

# PI control
controller = KP + KI/s
pc = series(controller, plant)
h = feedback(pc, 1)
t, y = step_response(h)
print(f'PI TF: {h}')
plt.subplot(2, 4, 5)
plt.plot(t, y)
reference = np.ones(len(t))
plt.plot(t, reference)
plt.title('PI')
plt.xlabel('time')
plt.ylabel('amplitude')

# PD control
controller = KP + KD*s
pc = series(controller, plant)
h = feedback(pc, 1)
t, y = step_response(h)
print(f'PD TF: {h}')
plt.subplot(2, 4, 6)
plt.plot(t, y)
reference = np.ones(len(t))
plt.plot(t, reference)
plt.title('PD')
plt.xlabel('time')
plt.ylabel('amplitude')

# ID control
controller = KI/s + KD*s
pc = series(controller, plant)
h = feedback(pc, 1)
t, y = step_response(h)
print(f'ID TF: {h}')
plt.subplot(2, 4, 7)
plt.plot(t, y)
reference = np.ones(len(t))
plt.plot(t, reference)
plt.title('ID')
plt.xlabel('time')
plt.ylabel('amplitude')

# PID control
controller = KP + KI/s + KD*s
pc = series(controller, plant)
h = feedback(pc, 1)
t, y = step_response(h)
print(f'PID TF: {h}')
plt.subplot(2, 4, 8)
plt.plot(t, y)
reference = np.ones(len(t))
plt.plot(t, reference)
plt.title('PID')
plt.xlabel('time')
plt.ylabel('amplitude')

# mark = marker.Marker()
# mark.set_scale(100)
# mark.linear(0, 1)
# for i in range(len(y)):
#     pos = y[i]
#     mark.set_pos(pos)

# plt.figure(2)
# poles of TF (denom = 0) have negative real parts -> stable
# any pole has positive real part -> unstable
# any pair of poles on imaginary axis -> oscillate
poles, zeros = pzmap(h, plot=False)
print(f'poles: {poles}')
print(f'zeros: {zeros}')
# plt.figure(3)
mag, phase, omega = bode_plot(h, plot=False)

plt.show()
