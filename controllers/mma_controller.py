import numpy as np
from .controller import Controller
from models.mmac_model1 import ManiuplatorModel1
from models.mmac_model2 import ManiuplatorModel2
from models.mmac_model3 import ManiuplatorModel3


class MMAController(Controller):
    def __init__(self, Tp):
        # TODO: Fill the list self.models with 3 models of 2DOF manipulators with different m3 and r3
        # I:   m3=0.1,  r3=0.05
        # II:  m3=0.01, r3=0.01
        # III: m3=1.0,  r3=0.3
        self.models = [ManiuplatorModel1(Tp), ManiuplatorModel2(Tp), ManiuplatorModel3(Tp)]
        self.i = 0
        self.Kd = 100
        self.Kp = 10
        self.v_prev = [0, 0]

    def choose_model(self, x):
        # TODO: Implement procedure of choosing the best fitting model from self.models (by setting self.i)
        # q1, q2, q1_dot, q2_dot = x

        e_m1 = x[2:] - self.models[0].M(x) @ self.v_prev + self.models[0].C(x) @ x[2:]
        e_m2 = x[2:] - self.models[1].M(x) @ self.v_prev + self.models[1].C(x) @ x[2:]
        e_m3 = x[2:] - self.models[2].M(x) @ self.v_prev + self.models[2].C(x) @ x[2:]

        prev_err = 999999999
        if prev_err > np.abs(e_m1[0]) + np.abs(e_m1[1]):
            prev_err = np.abs(e_m1[0]) + np.abs(e_m1[1])
            self.i = 0
        if prev_err > np.abs(e_m2[0]) + np.abs(e_m2[1]):
            prev_err = np.abs(e_m2[0]) + np.abs(e_m2[1])
            self.i = 1
        if prev_err > np.abs(e_m3[0]) + np.abs(e_m3[1]):
            prev_err = np.abs(e_m2[0]) + np.abs(e_m2[1])
            self.i = 2

        print("Choosen: " + str(self.i))

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        self.choose_model(x)

        q = x[:2]
        q_dot = x[2:]

        v = q_r_ddot + self.Kd * (q_r_dot - q_dot) + self.Kp * (q_r - q)
        M = self.models[self.i].M(x)
        C = self.models[self.i].C(x)
        u = M @ v + C @ q_dot
        self.v_prev = v

        return u
