import numpy as np
from observers.eso import ESO
from .controller import Controller
from models.manipulator_model import ManiuplatorModel


class ADRCJointController(Controller):
    def __init__(self, b, kp, kd, p, q0, Tp):
        self.b = b
        self.kp = kp
        self.kd = kd

        A = np.array([[0, 1, 0], [0, 0, 1], [0, 0, 0]])
        B = np.array([[0], [self.b], [0]])
        L = np.array([[3 * p], [3 * p ** 2], [p ** 3]])
        W = np.array([[1, 0, 0]])
        self.eso = ESO(A, B, W, L, q0, Tp)
        self.u_prev = 0
        self.model = ManiuplatorModel(Tp)

    def set_b(self, b):
        # TODO update self.b and B in ESO

        self.b = b
        # print(self.b)
        self.eso.set_B(np.array([[0], [self.b], [0]]))

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot, j_id):
        # TODO implement ADRC

        q = x[0]

        q_est, q_dot_est, f_est = self.eso.get_state()

        v = self.kp * (q_d - q) + self.kd * (q_d_dot - q_dot_est) + q_d_ddot  # q_d_ddot????
        u = (v - f_est) / self.b

        self.eso.update(q, u)

        self.u_prev = u

        M = self.model.M([x[0], x[1], 0., 0.])
        self.set_b(np.linalg.inv(M)[j_id, j_id])

        return u
