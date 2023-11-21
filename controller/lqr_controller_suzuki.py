import numpy as np
from tools.Mathfunction import Mathfunction as MF
from scipy import linalg
import csv
import pprint

# 初期設定
r = 2 / 3  # r(λ)=2/3Lp 長:0.3m
g = 9.8  # 重力加速度

A = np.array(
    [
        [0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, g, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, -g, 0, 0],
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

# 重みの決定   x  xd  β  y  yd γ  z  zd
Q = np.diag([1, 1, 1, 1, 1, 1, 1, 1])
R = np.diag([1, 1, 1])


# lqr法
def lqr(A, B, Q, R):
    P = linalg.solve_continuous_are(A, B, Q, R)  # Ricatti方程式を解いてる?
    K = linalg.inv(R).dot(B.T).dot(P)
    E = linalg.eigvals(A - B.dot(K))

    return P, K, E


P, K, E = lqr(A, Bl, Q, R)  # このKはMatlabの結果通りになってる

# P, K, E = control.lqr(A, Bl, Q, R)

# K = np.array([[1, 1.4518, 5.4272, 0, 0, 0, 0, 0], # Kを手打ち
#               [0, 0, 0, -1, -1.4518, 5.4272, 0, 0],
#               [0, 0, 0, 0, 0, 0, 1, 1.7321]])

# 目標値の設定（7行目は目標の高さz）
q_ref = np.array([[0], [0], [0], [0], [0], [0], [1], [0]])


class Lqr_Controller:
    def __init__(self, dt):
        self.dt = dt
        self.gravity_calcel = 9.81
        self.rad2deg = 180 / np.pi

        # * initialize input values
        self.input_acc = 0.0
        self.input_Wb = np.zeros(3)

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
        mode="position",
    ):
        self.Pref = P
        self.Vref = V
        self.Rref = R
        self.Eulerref = Euler
        self.Wbref = Wb
        self.Euler_rateref = Euler_rate

    def controller(self):
        q = np.array(
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

        q_div = q - q_ref
        u = -np.dot(K, q_div)
        ad = u[2, 0]
        a = ad + g + (g/2)*(self.Euler[1]*self.Euler[1] +
                            self.Euler[2]*self.Euler[2])

        # csvファイルにuの値を書き込む。
        with open("u.csv", "a") as f:
            writer = csv.writer(f)
            writer.writerow(u)
        with open("P.csv", "a") as f:
            writer = csv.writer(f)
            writer.writerow(u)

        # nominal acceleraion
        self.input_acc = a

        ox = u[1]
        oy = u[0]
        oz = 0

        # calculate input Body angular velocity
        self.input_Wb = np.array([[ox], [oy], [oz]], dtype=object)

        print(self.P)

        return self.input_acc, self.input_Wb
