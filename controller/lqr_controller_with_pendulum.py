import numpy as np
from tools.Mathfunction import Mathfunction as MF
from scipy import linalg
from control.matlab import ctrb
from numpy.linalg import matrix_rank


class Lqr_Controller:
    def __init__(self, dt):
        self.dt = dt
        self.gravity_calcel = 9.81
        self.rad2deg = 180 / np.pi

        # * initialize input values
        self.input_acc = 0.0
        self.input_Wb = np.zeros(3)
        # 初期設定
        Lp = 0.3  # 振子の全長 [m]
        r = (2/3)*Lp  # r(λ)=2/3Lp 長:0.3m
        self.g = 9.8  # 重力加速度

        self.A = np.array(
            [
                [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, self.g, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, self.g/r, 0, -self.g/r, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, -self.g, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, self.g/r, 0, -self.g/r, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            ]
        )

        self.Bl = np.array(
            [
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [1, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [0, 1, 0],
                [0, 0, 0],
                [0, 0, 1],
            ]
        )

        # 目標値の設定（7行目が目標の高さz）
        self.q_ref = np.array([[0], [0], [0], [0], [0], [0], [
            0], [0], [0], [0], [0.5], [0]])

        # 重みの決定   [x  xd  β  y  yd γ  z  zd]
        self.Q = np.diag([15, 1, 1, 1, 50, 15, 1, 1, 1, 50, 80, 1])  # 振子有り
        self.R = np.diag([50, 50, 1])  # 振子有り

        # 可制御性の判定
        # C = ctrb(self.A, self.Bl)
        # if matrix_rank(C) == len(self.A):
        #     print("The system is controllable")
        #     print("rankC = ", matrix_rank())
        #     print("lenA = ", len(self.A))
        # else:
        #     print("可制御でない")
        #     print("====================================")
        #     print(C)
        #     print(matrix_rank(C))
        #     print(len(self.A))

        self.P, self.K, self.E = self.lqr(self.A, self.Bl, self.Q, self.R)

    # lqr法
    def lqr(self, A, B, Q, R):
        P = linalg.solve_continuous_are(A, B, Q, R)  # Ricatti方程式を解いてる?
        K = linalg.inv(R).dot(B.T).dot(P)
        E = linalg.eigvals(A - B.dot(K))

        return P, K, E

    def set_dt(self, dt):
        self.dt = dt

    def set_state(self, drone):
        self.P = drone.P.now
        self.V = drone.V.now
        self.A = drone.A
        self.R = drone.R.now
        self.Euler = drone.Euler.now

    def set_reference(
        self,
        P=np.array([0.0, 0.0, 1.0]),
        V=np.array([0.0, 0.0, 0.0]),
        R=np.eye(3),
        Euler=np.array([0.0, 0.0, 0.0]),
        Wb=np.array([0.0, 0.0, 0.0]),
        Euler_rate=np.array([0.0, 0.0, 0.0]),
        Pendulum_angle=np.array([0.0, 0.0]),
        Pendulum_angle_V=np.array([0.0, 0.0]),
        mode="position",
    ):
        self.Pref = P
        self.Vref = V
        self.Rref = R
        self.Eulerref = Euler
        self.Wbref = Wb
        self.Euler_rateref = Euler_rate
        self.Pendulum_angle = Pendulum_angle
        self.Pendulum_angle_V = Pendulum_angle_V

    def controller(self):
        q = np.array(
            [
                [self.P[0]],
                [self.V[0]],
                [self.Pendulum_angle[0]],
                [self.Pendulum_angle_V[0]],
                [self.Euler[1]],
                [self.P[1]],
                [self.V[1]],
                [self.Pendulum_angle[1]],
                [self.Pendulum_angle_V[1]],
                [self.Euler[0]],
                [self.P[2]],
                [self.V[2]],
            ]
        )

        q_div = q - self.q_ref
        u = -np.dot(self.K, q_div)
        ad = u[2, 0]
        # 1/2にかかっているg不要かも
        a = ad + self.g + (self.g/2)*(self.Euler[1]*self.Euler[1] +
                                      self.Euler[0]*self.Euler[0])
        ac = a/100

        # nominal acceleraion
        self.input_acc = a

        ox = u[1]
        oy = u[0]
        oz = -10*self.Euler[2]

        # calculate input Body angular velocity
        self.input_Wb = np.array([[ox], [oy], [oz]], dtype=object)

        print(self.P, self.Euler, self.V)

        return self.input_acc, self.input_Wb
