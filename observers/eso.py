from copy import copy
import numpy as np


class ESO:
    def __init__(self, A, B, W, L, state, Tp):
        self.A = A
        self.B = B
        self.W = W
        self.L = L
        self.state = np.pad(np.array(state), (0, A.shape[0] - len(state)))
        self.Tp = Tp
        self.states = []

    def set_B(self, B):
        self.B = B

    def update(self, q, u):
        self.states.append(copy(self.state))

        z_est = np.reshape(self.state, (len(self.state), 1))
        z_dot_est = self.A @ z_est + self.B @ np.atleast_2d(u) + (self.L @ (q - self.W @ z_est))
        self.state += (self.Tp * np.reshape(z_dot_est, (1, 3)))[0]

    def get_state(self):
        return self.state
