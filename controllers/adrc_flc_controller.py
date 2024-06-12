import numpy as np

from observers.eso import ESO
from .adrc_joint_controller import ADRCJointController
from .controller import Controller
from models.manipulator_model import ManiuplatorModel


class ADRFLController(Controller):
    def __init__(self, Tp, q0, Kp, Kd, p):
        self.model = ManiuplatorModel(Tp)
        self.Kp = Kp
        self.Kd = Kd
        self.b = 1

        self.A = np.array([[0, 1, 0], [0, 0, 1], [0, 0, 0]])
        self.B = np.array([[0], [self.b], [0]])
        self.L = np.array([[6 * p], [20 * p ** 2], [6 * p ** 3]])
        self.W = np.array([[1, 0, 0]])

        self.eso = ESO(self.A, self.B, self.W, self.L, q0, Tp)
        self.update_params(q0[:2], q0[2:])

    def update_params(self, q, q_dot):
        ### TODO Implement procedure to set eso.A and eso.B
        self.eso.A = None
        self.eso.B = None

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):
        ### TODO implement centralized ADRFLC
        return NotImplementedError
