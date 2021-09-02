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
x0_KST = init_kst(initialState, alpha0)  # initial state for kinematic single-track trailer model


# ********* Simulate maneuver ************
# cornering left
t = numpy.arange(0, tFinal, 0.1)
v_delta = 0.1           # steering angle velocity
a_long = 0              # longitudinal acceleration
u = [v_delta, a_long]   # input vector

# simulate kinematic single-track trailer model
x_left_kst = odeint(func_KST, x0_KST, t, args=(u, p))

time = 1
with open("trajectory", 'w') as traj:
    traj.write("<trajectory>")
    for sol in x_left_kst:
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
        traj.write("\n    </hitch>")
        traj.write("\n  </state>")
        time += 1
    traj.write("\n</trajectory>")

# ********* Plot results ****************
# plot results
# positions
title('positions')
plt.plot([tmp[0] for tmp in x_left_kst], [tmp[1] for tmp in x_left_kst])
plt.show()
# steering angle
title('steering angle')
plt.plot(t, [tmp[2] for tmp in x_left_kst])
plt.show()
# velocity
title('velocity')
plt.plot(t, [tmp[3] for tmp in x_left_kst])
plt.show()
# yaw angle truck
title('yaw angle truck')
plt.plot(t, [tmp[4] for tmp in x_left_kst])
plt.show()
# hitch angle
title('hitch angle')
plt.plot(t, [tmp[5] for tmp in x_left_kst])
plt.show()
