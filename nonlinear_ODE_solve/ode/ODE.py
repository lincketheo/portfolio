import numpy as np
import matplotlib.pyplot as plt
import warnings
from mpl_toolkits.mplot3d import Axes3D
from math import log10, floor
import sys


# A Linear Equation Class
class ODE:

    ##
    # @brief Create a new non linear Equation
    #
    # @param state_func The state function - takes in state, t 
    # @param size Size of state space 
    # @param tolerance For adaptive time step
    # @param start_state Starting state of the state system
    # @param time_step Starting or stagnant time step
    def __init__(self, state_func, size, tolerance = 0.001, start_state = None, time_step = 0.001):


        # Error handling for state_function
        if 'state' not in state_func.__code__.co_varnames:
            raise ValueError('Invalid function passed to ODE, must contain a \'state\' vector parameter')

        if 't' not in state_func.__code__.co_varnames:
            raise ValueError('Invalid function passed to ODE, must contain a \'t\' time parameter')

        if state_func.__code__.co_argcount > 2 and 'self' not in state_func.__code__.co_varnames:
            warnings.warn("State function passed takes more than 2 parameters while ODE assumes it takes 2")

        # Update intrinsic properties
        self.size = size
        self.func = state_func
        self.h = time_step
        self.time = 0.0
        self.tolerance = tolerance

        # Give self a start state (must mach size)
        if start_state == None or len(start_state) != self.size:
            self.starting_state = np.zeros(self.size, dtype=np.float64)
            self.state = np.zeros(self.size, np.float64)
        else:
            self.starting_state = np.array(start_state, dtype=np.float64)
            self.state = np.array(start_state, dtype=np.float64)



    ##
    # @brief RK4 single step
    #
    # @param h delta t
    # @param t Current time
    # @param x Value of the state
    #
    # @return t (next time), state (next state)
    def de_step_rk4(self, h, t, x):
        k1 = h * self.func(state = x, t = t)
        k2 = h * self.func(state = x + k1 / 2, t = t + h / 2)
        k3 = h * self.func(state = x + k2 / 2, t = t + h / 2)
        k4 = h * self.func(state = x + k3, t = t + h)

        return t + h, (x + (k1 + 2 * k2 + 2 * k3 + k4) / 6)

    ##
    # @brief Step forward once
    #
    # @param adaptive_t If true, adapts h, otherwise, keeps h same
    def step(self, adaptive_t):
        if adaptive_t:

            # The maximum recursive depth to attach an upper bound on error propigation
            # (each step can do no more than i updates)
            i = 10
            
            # One step forward
            t1, x1 = self.de_step_rk4(self.h, self.time, self.state)

            # Two steps forward
            t2, x2 = self.de_step_rk4(self.h / 2, self.time, self.state)
            t3, x3 = self.de_step_rk4(self.h / 2, t2, x2)

            # The error of h/2 vs h
            error = np.linalg.norm(x3 - x1)

            if(error > self.tolerance):

                # Continue halving self.h until error is 
                # within the tolerance
                while error > self.tolerance and i > 0:
                    self.h = self.h / 2

                    # One step forward
                    t1, x1 = self.de_step_rk4(self.h, self.time, self.state)

                    # Two steps forward
                    t2, x2 = self.de_step_rk4(self.h / 2, self.time, self.state)
                    t3, x3 = self.de_step_rk4(self.h / 2, t2, x2)

                    error = np.linalg.norm(x3 - x1)
                    i -= 1

                if i <= 0:
                    warnings.warn("Maximum depth in greater than error bound exceeded")

                # Pick the best step no matter what
                self.time, self.state = t3, x3

            elif(error < self.tolerance):

                # Pick the best step
                self.time, self.state = t3, x3

                while error < self.tolerance and i > 0:
                    self.h = self.h * 2

                    # One step forward
                    t1, x1 = self.de_step_rk4(self.h, self.time, self.state)

                    # Two steps forward
                    t2, x2 = self.de_step_rk4(self.h / 2, self.time, self.state)
                    t3, x3 = self.de_step_rk4(self.h / 2, t2, x2)

                    error = np.linalg.norm(x3 - x1)
                    i -= 1

                if i <= 0:
                    warnings.warn("Maximum depth in less than error bound exceeded")

        else:
            # Very simple update step
            t1, x1 = self.de_step_rk4(self.h, self.time, self.state)
            self.time, self.state = t1, x1


    ##
    # @brief Generate a list of state variables
    #
    # @param n 
    # @param adaptive_t
    #
    # @return 
    def ode(self, n, adaptive_t = False, reset_on_end = True, verbose=False):
        time = np.zeros(n)
        states = np.zeros(n * self.size, dtype=np.float64).reshape(self.size, n)

        for i in range(n):
            self.step(adaptive_t)
            time[i] = self.time
            states[:,i] = self.state
            if verbose:
                print(self.time, self.state)

        if reset_on_end:
            self.reset()

        return time, states


    ##
    # @brief Temporal slice generator
    #
    # @param n number of elements to compute (temporal slice elements that is)
    # @param T The periodic T multiple to compute
    # @param max The maximum number of elements to compute (if n is too large)
    def temporal_slice(self, n, T, reset_on_end = False, verbose = False, max_states = 10000000, ode_states = False):

        # The resolution to calculate 
        resolution = abs(floor(log10(self.h)))

        states = []
        states_osc = []


        # Iterator count for states
        i = 0

        # Number of elements found
        found = 0

        prev_t = 0

        while i < max_states and found < n:

            prev_t = self.time

            i += 1
            # Update ode
            self.step(False)

            # Append current states to state vector
            if ode_states:
                states.append(self.state)

            # Test for intersection
            if prev_t % T > self.time % T:
                if verbose:
                    print("Element found: ", self.time, T)

                h_temp = self.time - (floor(self.time / T) * T)
                t, x = self.de_step_rk4(-h_temp, self.time, self.state)
                states_osc.append(x)
                found += 1


        if len(states_osc) != n:
            warnings.warn(f"Did not create enough elements: itterator = {i}")

        if ode_states:
            return np.array(states_osc, dtype=np.float64), np.array(states, dtype=np.float64)

        else:
            return np.array(states_osc, dtype=np.float64)


    def reset(self, start_state = None):
        if start_state == None or len(start_state) != self.size:
            self.state = self.starting_state
        else:
            self.starting_state = np.array(start_state, dtype=np.float64)
            self.state = np.array(start_state, dtype=np.float64)

        self.time = 0.0
