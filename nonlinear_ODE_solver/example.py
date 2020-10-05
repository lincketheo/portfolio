from ode import *
import matplotlib.pyplot as plt
from numpy import pi, sqrt
from mpl_toolkits.mplot3d import Axes3D

# An example application of ODE
if __name__ == '__main__':

    # The drive frequency
    alpha = 90

    # Create a simple pendulum system
    p = lorentz(10, 28, 8/3)

    # Create a new ode with the pendulum state function 
    a = ODE(state_func = p.state_func, size=3, start_state = [1, 1, 1], time_step=0.001)

    
    # Get the time and states of every part of the pendulum
    time, states = a.ode(5000, adaptive_t = True, reset_on_end = True, verbose = False)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(states[0], states[1], states[2], linewidth=0.1)
    plt.show()

