import numpy as np


##
# @brief Rossler system of equations
class rossler:
    def __init__(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c

    def state_func(self, state, t):
        xdot = -(state[1] + state[2])
        ydot = state[0] + a * state[1]
        zdot = b + state[2] * (state[0] - c)
        return np.array([xdot, ydot, zdot], dtype=np.float64)


##
# @brief Lorentz system of equations
class lorentz:
    def __init__(self, a, r, b):
        self.a = np.float64(a)
        self.r = np.float64(r)
        self.b = np.float64(b)

        self.jacobian = np.array(
                [[-a, a, 0], 
                 [0, -1, 0], 
                 [0, 0, -b]], dtype=np.float64)

        # A place to store resulting vectors
        self.result = np.zeros(12, dtype=np.float64)

    def state_func(self, state, t):
        xdot = self.a * (state[1] - state[0])
        ydot = self.r * state[0] - state[0] - state[0] * state[2]
        zdot = state[0] * state[1] - self.b * state[2]
        return np.array([xdot, ydot, zdot], dtype=np.float64)

    def variational_state_func(self, state, t):
        self.update_jacobian(state)

        self.result[-9:] = np.matmul(self.jacobian, state[-9:].reshape(3, 3)).reshape(1, 9)
        self.result[:3] = self.state_func(state[:3], t)
        return self.result

    def update_jacobian(self, state):
        self.jacobian[1][0] = self.r - state[2]
        self.jacobian[1][2] = -state[0]
        self.jacobian[2][0] = state[1]
        self.jacobian[2][1] = state[0]



##
# @brief Pendulum system of equations
class pendulum:
    def __init__(self, beta, l, m, g, A, alpha):
        self.beta = beta
        self.l = l
        self.m = m
        self.g = g
        self.A = A
        self.alpha = alpha
        self.natfreq = np.sqrt(self.g / self.l) / (2 * np.pi)

    def state_func(self, state, t):
        thetadot = state[1]
        omegadot = (self.A * np.cos(self.alpha * t) - self.beta * self.l * state[1] - self.m * self.g * np.sin(state[0])) / (self.m * self.l)
        return np.array([thetadot, omegadot], dtype=np.float64)
