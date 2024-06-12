import numpy as np
from observers.eso import ESO
from .controller import Controller
from models.manipulator_model import ManiuplatorModel


class ADRCJointController(Controller):
    def __init__(self, b, kp, kd, p, q0, Tp):
        self.b = b
        self.kp = kp
        self.kd = kd

        self.A = np.array([[0, 1, 0], [0, 0, 1], [0, 0, 0]])
        self.B = np.array([[0], [self.b], [0]])
        self.L = np.array([[6 * p], [20 * p ** 2], [6 * p ** 3]])
        self.W = np.array([[1, 0, 0]])

        self.eso = ESO(self.A, self.B, self.W, self.L, q0, Tp)
        self.u_prev = 0
        self.model = ManiuplatorModel(Tp)

    def set_b(self, b):

        self.b = b
        self.eso.set_B(np.array([[0], [self.b], [0]]))

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot, j_id):

        q = x[0]

        q_est, q_dot_est, f_est = self.eso.get_state()

        v = self.kp * (q_d - q) + self.kd * (q_d_dot - q_dot_est)
        u = (v - f_est) / self.b

        self.eso.update(q, u)

        self.u_prev = u

        M = self.model.M([x[0], x[1], 0., 0.])
        self.set_b(np.linalg.inv(M)[j_id, j_id])

        return u
