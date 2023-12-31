# from https://physics.stackexchange.com/questions/256468/model-formula-for-bouncing-ball

from math import sqrt
import matplotlib.pyplot as plt

h0 = 5  # m
v = -10  # m/s, current velocity
g = 9.81  # m/s/s
t = 0  # starting time
dt = 0.001  # time step
rho = 0.75  # coefficient of restitution
tau = 0.10  # contact time for bounce
hmax = h0  # keep track of the maximum height
h = h0
hstop = 0.01  # stop when bounce is less than 1 cm
freefall = True  # state: freefall or in contact
t_last = -sqrt(2 * h0 / g)  # time we would have launched to get to h0 at t=0
vmax = sqrt(2 * hmax * g)
H = []
T = []
while hmax > hstop:
    if freefall:
        hnew = h + v * dt - 0.5 * g * dt * dt
        if hnew < 0:
            t = t_last + 2 * sqrt(2 * hmax / g)
            freefall = False
            t_last = t + tau
            h = 0
        else:
            t = t + dt
            v = v - g * dt
            h = hnew
    else:
        t = t + tau
        vmax = vmax * rho
        v = vmax
        freefall = True
        h = 0
    hmax = 0.5 * vmax * vmax / g
    H.append(h)
    T.append(t)

print("stopped bouncing at t=%.3f\n" % (t))

plt.figure()
plt.plot(T, H)
plt.xlabel('time')
plt.ylabel('height')
plt.title('bouncing ball')
plt.show()
