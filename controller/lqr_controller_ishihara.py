import csv
import numpy as np
from tools.Mathfunction import Mathfunction as MF
from scipy import linalg


class Lqr_Controller:
    def __init__(self, dt):
        self.dt = dt
        self.r = 2 / 3  # r(λ)=2/3Lp 長:0.3m
        self.g = 9.8  # 重力加速度

        # * initialize input values
        self.input_acc = 0.0
        self.input_Wb = np.zeros(3)

        # self.lqr_init() 一旦呼び出さないようにする

        self.K = np.array([
            [1.00000000000000, 1.45175395145263, 5.42718872423573, 8.97157015039534e-17,
                2.36934578759692e-16, -1.51758575511169e-15, -1.83826241986066e-16, 3.25349858845081e-16],
            [-4.54758256147348e-16, -2.68464342807294e-16, 1.87287620071113e-16, -0.999999999999999, -
                1.36885555678166, 6.42217668469037, 1.48180184268838e-15, 3.30898740618047e-16],
            [1.79859175550358e-16, 4.09738600794711e-16, -1.25760985716651e-16, 5.83691553045932e-17,
                1.23204693030713e-16, -2.31044461374617e-16, 1.00000000000000, 1.73205080756888]
        ])

        self.q_ref = np.array([[0], [0], [0], [0], [0], [0], [1], [0]])

    def set_dt(self, dt):
        self.dt = dt

    def lqr_init(self):
        # 初期設定
        self.r = 2 / 3  # r(λ)=2/3Lp 長:0.3m
        self.g = 9.8  # 重力加速度

        A = np.array(
            [
                [0, 1, 0, 0, 0, 0, 0, 0],
                [0, 0, self.g, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, -self.g, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0, 0, 0],
            ]
        )

        Bl = np.array(
            [
                [0, 0, 0],
                [0, 0, 0],
                [1, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [0, 1, 0],
                [0, 0, 0],
                [0, 0, 1],
            ]
        )

        # 重みの決定   Q=[x, xd, β, y, yd, γ, z, zd]
        Q = np.diag([1, 1, 1, 1, 1, 1, 1, 1])
        R = np.diag([1, 1, 1])

        # 目標値の設定（7行目は目標の高さz）
        # self.q_ref = np.array([[0], [0], [0], [0], [0], [0], [1], [0]])

        # self.P, self.K, self.E = self.lqr(A, Bl, Q, R)  # このKはMatlabの結果通りになってる

    def lqr(self, A, B, Q, R):
        P = linalg.solve_continuous_are(A, B, Q, R)  # Ricatti方程式を解いてる?
        K = linalg.inv(R).dot(B.T).dot(P)
        E = linalg.eigvals(A - B.dot(K))

        return P, K, E

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
        mode="position",
    ):
        self.Pref = P
        self.Vref = V
        self.Rref = R
        self.Eulerref = Euler
        self.Wbref = Wb
        self.Euler_rateref = Euler_rate

    def controller(self):
        self.q_now = np.array(
            [
                [self.P[0]],
                [self.V[0]],
                [self.Euler[1]],
                [self.P[1]],
                [self.V[1]],
                [self.Euler[2]],
                [self.P[2]],
                [self.V[2]],
            ]
        )

        q_div = np.array(self.q_now - self.q_ref.squeeze())
        self.input = -self.K @ q_div
    # [3rows, 1cols]

        # self.input = -np.dot(self.K, q_div)
        # self.input = -np.dot(self.K, ((self.q_now - self.q_ref.squeeze())))
        self.uncalculated_Zacc = self.input[2, 0]
        self.input_Zacc = self.input[2, 0] + self.g + (self.g/2)*(self.Euler[1]*self.Euler[1] +
                                                                  self.Euler[2]*self.Euler[2])

        # nominal acceleration
        # self.input_acc = a

        wx = self.input[0, 0]
        wy = self.input[1, 0]
        wz = 0

        # calculate input Body angular velocity
        self.input_Wb = np.array([wx, wy, wz], dtype=object)

        return self.input_Zacc, self.input_Wb
