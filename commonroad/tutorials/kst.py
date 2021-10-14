from scipy.integrate import odeint
import numpy
import matplotlib.pyplot as plt
from matplotlib.pyplot import title, legend
import math

from vehiclemodels.parameters_vehicle4 import parameters_vehicle4
from vehiclemodels.init_kst import init_kst
from vehiclemodels.vehicle_dynamics_kst import vehicle_dynamics_kst


def func_KST(x, t, u, p):
    f = vehicle_dynamics_kst(x, u, p)
    return f


# load parameters
p = parameters_vehicle4()
g = 9.81  # [m/s^2]

# set simulating time
tStart = 0  # start time
tFinal = 4  # start time

# set intial state
delta0 = 0
vel0 = 10
Psi0 = 0
dotPsi0 = 0
beta0 = 0
sx0 = 0
sy0 = 0
alpha0 = 0
initialState = [sx0, sy0, delta0, vel0, Psi0, dotPsi0, beta0]  # initial state for simulation
x0_KST = init_kst(initialState, [0, 0])  # initial state for kinematic single-track trailer model


# ********* Simulate maneuver ************
# cornering left
t = numpy.arange(0, tFinal, 0.1)
v_delta = 0.2           # steering angle velocity
a_long = 0              # longitudinal acceleration
u = [v_delta, a_long]   # input vector

# simulate kinematic single-track trailer model
x_left_kst = odeint(func_KST, x0_KST, t, args=(u, p))

last_res = x_left_kst[39]

iniState = [last_res[0], last_res[1], -last_res[2], last_res[3], last_res[4], last_res[5], last_res[6]]
x0_KST2 = init_kst(iniState, [last_res[5], last_res[6]])
t1 = numpy.arange(tFinal, 2 * tFinal, 0.1)
v_delta = 0.3           # steering angle velocity
a_long = 0              # longitudinal acceleration
u = [v_delta, a_long]   # input vector
# simulate kinematic single-track trailer model
x_left_kst1 = odeint(func_KST, x0_KST2, t1, args=(u, p))

results = []
for sol in x_left_kst:
    results.append(sol)
for sol in x_left_kst1:
    results.append(sol)

time = 1
with open("trajectory", 'w') as traj:
    traj.write("<trajectory>")
    for sol in results:
        traj.write("\n  <state>")
        traj.write("\n    <position>")
        traj.write("\n      <point>")
        traj.write("\n        <x>" + str(sol[0]) + "</x>")
        traj.write("\n        <y>" + str(sol[1]) + "</y>")
        traj.write("\n      </point>")
        traj.write("\n    </position>")
        traj.write("\n    <orientation>")
        traj.write("\n      <exact>" + str(sol[4]) + "</exact>")
        traj.write("\n    </orientation>")
        traj.write("\n    <time>")
        traj.write("\n      <exact>" + str(time) + "</exact>")
        traj.write("\n    </time>")
        traj.write("\n    <velocity>")
        traj.write("\n      <exact>" + str(sol[3]) + "</exact>")
        traj.write("\n    </velocity>")
        traj.write("\n    <acceleration>")
        traj.write("\n      <exact>" + str(a_long) + "</exact>")
        traj.write("\n    </acceleration>")
        traj.write("\n    <hitch>")
        traj.write("\n      <exact>" + str(sol[5]) + "</exact>")
        traj.write("\n      <exact>" + str(sol[6]) + "</exact>")
        traj.write("\n    </hitch>")
        traj.write("\n  </state>")
        time += 1
    traj.write("\n</trajectory>")

# ********* Plot results ****************
# plot results
# positions
timeeee = []
for t0 in t:
    timeeee.append(t0)
for t0 in t1:
    timeeee.append(t0)
title('positions')
plt.plot([tmp[0] for tmp in results], [tmp[1] for tmp in results])
plt.show()
# steering angle
title('steering angle')
plt.plot(timeeee, [tmp[2] for tmp in results])
plt.show()
# velocity
title('velocity')
plt.plot(timeeee, [tmp[3] for tmp in results])
plt.show()
# yaw angle truck
title('yaw angle truck')
plt.plot(timeeee, [tmp[4] for tmp in results])
plt.show()
# hitch angle
title('hitch angle1')
plt.plot(timeeee, [tmp[5] for tmp in results])
plt.show()
# hitch angle
title('hitch angle2')
plt.plot(timeeee, [tmp[6] for tmp in results])
plt.show()
