import numpy as np
from .controller import Controller
from models.mmac_model1 import ManiuplatorModel1
from models.mmac_model2 import ManiuplatorModel2
from models.mmac_model3 import ManiuplatorModel3


class MMAController(Controller):
    def __init__(self, Tp):
        # I:   m3=0.1,  r3=0.05
        # II:  m3=0.01, r3=0.01
        # III: m3=1.0,  r3=0.3
        self.models = [ManiuplatorModel1(Tp), ManiuplatorModel2(Tp), ManiuplatorModel3(Tp)]
        self.i = 0
        self.Kd = 3
        self.Kp = 3
        self.u_prev = [0, 0]
        self.x_prev = [0, 0, 0, 0]
        self.first = True
        self.Tp = Tp

    def choose_model(self, x):
        # q1, q2, q1_dot, q2_dot = x

        biggest_error = 99999

        for i in range(0, 3):
            q_ddot = np.linalg.solve(self.models[i].M(self.x_prev), self.u_prev - np.dot(self.models[i].C(self.x_prev), self.x_prev[2:]))
            q_dot = self.x_prev[2:] + q_ddot * self.Tp
            q = x[:2] + q_dot * self.Tp

            x_est = np.concatenate([q, q_dot], axis=0)

            err = abs(x_est[0] - x[0]) + abs(x_est[1] - x[1]) + abs(x_est[2] - x[2]) + abs(x_est[3] - x[3])

            if err < biggest_error:
                biggest_error = err
                self.i = i

        print("Choosen: " + str(self.i))

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        if self.first:
            self.first = False
            self.x_prev = x

        self.choose_model(x)

        q = x[:2]
        q_dot = x[2:]

        v = q_r_ddot + self.Kd * (q_r_dot - q_dot) + self.Kp * (q_r - q)
        M = self.models[self.i].M(x)
        C = self.models[self.i].C(x)
        u = M @ v + C @ q_dot

        self.u_prev = u
        self.x_prev = x

        return u
