import numpy as np
from models.manipulator_model import ManiuplatorModel
from .controller import Controller


class FeedbackLinearizationController(Controller):
    def __init__(self, Tp):
        self.model = ManiuplatorModel(Tp)
        self.Kd = 1
        self.Kp = 1

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        """
        Please implement the feedback linearization using self.model (which you have to implement also),
        robot state x and desired control v.
        """

        q1, q2, q1_dot, q2_dot = x
        
        # Bez sprzezenia zwrotnego
        # tau = self.model.M(x) @ q_r_ddot + self.model.C(x) @ x[2:]
        
        # Ze sprzezeniem zwrotnym
        v = q_r_ddot + self.Kd * (q_r_dot - x[2:]) + self.Kp * (q_r - x[:2])
        tau = self.model.M(x) @ v + self.model.C(x) @ x[2:]
        
        return tau
