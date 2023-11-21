from tools.Mathfunction import Mathfunction
from Exp_Controller.Trajectory import Trajectory
import numpy as np
import sys
import math
from scipy import linalg

sys.path.append("../")


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

# 重みの決定
Q = np.diag([1, 1, 1, 1, 1, 1, 1, 1])
R = np.diag([1, 1, 1])


# lqr法
def lqr(A, B, Q, R):
    P = linalg.solve_continuous_are(A, B, Q, R)
    K = linalg.inv(R).dot(B.T).dot(P)
    E = linalg.eigvals(A - B.dot(K))

    return P, K, E


P, K, E = lqr(A, Bl, Q, R)

# 目標値の設定（7行目は目標の高さz）
q_ref = np.array([[0], [0], [0], [0], [0], [0], [0.5], [0]])


class Mellinger(Mathfunction):
    def __init__(self, dt):
        self.dt = dt

    def mellinger_init(self):
        print("Init Mellinger Controller")

        # init trajectory

        self.kp = np.array([4.3, 4.3, 3.0])
        self.kv = np.array([3.8, 3.8, 2.5])
        self.ki = np.array([0.0, 0.0, 0.0])
        self.ka = np.array([0.0, 0.0, 0.0])
        self.kR = np.array([8.5, 8.5, 0.5])

        self.Euler_nom = np.array([0.0, 0.0, 0.0])
        self.Euler_rate_nom = np.array([0.0, 0.0, 0.0])
        self.traj_W = np.zeros(3)

        self.input_acc = 0.0
        self.input_Wb = np.zeros(3)

        self.trajectory = Trajectory()

    def set_reference(self, traj_plan, t, tmp_P=np.zeros(3)):
        self.trajectory.set_clock(t)
        self.trajectory.set_traj_plan(traj_plan)

        self.tmp_pos = np.zeros(3)
        self.Pi = np.zeros(3)
        self.ki = np.array([0.0, 0.0, 0.0])
        # * set takeoff position for polynominal land trajectory
        if traj_plan == "takeoff" or traj_plan == "takeoff_50cm":
            self.tmp_pos = tmp_P
            self.ki[2] = 0.5
        # * set landing position for polynominal land trajectory
        elif traj_plan == "land" or traj_plan == "land_50cm":
            self.tmp_pos = tmp_P
            self.ki[2] = 1.5
        # * set stop position when stop tracking trajectory
        elif traj_plan == "stop":
            self.tmp_pos = tmp_P

    def set_state(self, state):
        self.P = state.P
        self.V = state.Vfiltered
        self.R = state.R
        self.Euler = state.Euler

    def Position_controller(self):
        # set trajectory of each state
        traj_pos = self.trajectory.traj_pos + self.tmp_pos
        traj_vel = self.trajectory.traj_vel
        traj_acc = self.trajectory.traj_acc

        # calcurate nominal acceleration
        self.Pi += (traj_pos - self.P) * self.dt
        self.ref_acc = (
            self.kp * (traj_pos - self.P)
            + self.kv * (traj_vel - self.V)
            + self.ki * self.Pi
            + traj_acc
        )

        """
        # nominal acceleraion
        self.input_acc = np.dot(
            self.ref_acc, np.matmul(self.R, np.array([0.0, 0.0, 1.0]))
        )
        """

        # calcurate nominal acceleration
        # q = [x,dx,θ,dθ,β,y,dy,Φ,dΦ,γ,z,dz]
        # 振子なしの場合、q = [x,dx,β,y,dy,γ,z,dz]
        """
        x = self.P[0,0]
        y = self.P[1,0]
        z = self.P[2,0]
        dx = self.V[0,0]
        dy = self.V[1,0]
        dz = self.V[2,0]
        b = self.Euler[0,0]
        c = self.Euler[1,0]
        
        q = np.array([[x],
                      [dx],
                      [b],
                      [y],
                      [dy],
                      [c],
                      [z],
                      [dz]])
        """
        """
        #多分[1,0]とかでなく、[0,1]
        q = np.array([[self.P[0,0]],
                      [self.V[0,0]],
                      [self.Euler[0,0]],
                      [self.P[1,0]],
                      [self.V[1,0]],
                      [self.Euler[1,0]],
                      [self.P[2,0]],
                      [self.V[2,0]]])
        """
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

        # nominal acceleraion
        self.input_acc = a

    def Attitude_controller(self):
        # set trajectory of each state
        traj_acc = self.trajectory.traj_acc
        traj_jer = self.trajectory.traj_jer
        traj_yaw = self.trajectory.traj_yaw
        traj_yaw_rate = self.trajectory.traj_yaw_rate

        # calculate nominal Rotation matrics
        traj_R = np.zeros((3, 3))
        traj_Rxc = np.array([np.cos(traj_yaw), np.sin(traj_yaw), 0.0])
        traj_Ryc = np.array([-np.sin(traj_yaw), np.cos(traj_yaw), 0.0])
        traj_Rz = self.ref_acc / np.linalg.norm(self.ref_acc)
        traj_Rx = np.cross(traj_Ryc, traj_Rz) / np.linalg.norm(
            np.cross(traj_Ryc, traj_Rz)
        )
        traj_Ry = np.cross(traj_Rz, traj_Rx) / np.linalg.norm(
            np.cross(traj_Rz, traj_Rx)
        )
        # traj_Ry = np.cross(traj_Rz, traj_Rxc)
        # traj_Rx = np.cross(traj_Ry, traj_Rz)

        traj_R[:, 0] = traj_Rx
        traj_R[:, 1] = traj_Ry
        traj_R[:, 2] = traj_Rz

        # calculate nominal Angular velocity
        traj_wy = np.dot(traj_Rx, traj_jer) / np.dot(traj_Rz, self.ref_acc)
        traj_wx = -np.dot(traj_Ry, traj_jer) / np.dot(traj_Rz, self.ref_acc)
        traj_wz = (
            traj_yaw_rate * np.dot(traj_Rxc, traj_Rx)
            + traj_wy * np.dot(traj_Ryc, traj_Rz)
        ) / np.linalg.norm(np.cross(traj_Ryc, traj_Rz))
        self.traj_W[0] = traj_wx
        self.traj_W[1] = traj_wy
        self.traj_W[2] = traj_wz

        # calculate nominal Angular velocity
        # q = [x,dx,θ,dθ,β,y,dy,Φ,dΦ,γ,z,dz]
        # 振子なしの場合、q = [x,dx,β,y,dy,γ,z,dz]
        """
        x = self.P[0,0]
        y = self.P[1,0]
        z = self.P[2,0]
        dx = self.V[0,0]
        dy = self.V[1,0]
        dz = self.V[2,0]
        b = self.Euler[0,0]
        c = self.Euler[1,0]
        
        q = np.array([[x],
                      [dx],
                      [b],
                      [y],
                      [dy],
                      [c],
                      [z],
                      [dz]])
        """
        """
        q = np.array(
            [
                [self.P[0, 0]],
                [self.V[0, 0]],
                [self.Euler[0, 0]],
                [self.P[1, 0]],
                [self.V[1, 0]],
                [self.Euler[1, 0]],
                [self.P[2, 0]],
                [self.V[2, 0]],
            ]
        )
        """
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

        # ωxとωy(ωxとωyが逆の可能性あり)
        """
        ox = u[1, 0]
        oy = u[0, 0]
        oz = 0
        """
        ox = u[1]
        oy = u[0]
        oz = 0

        """
        # calculate input Body angular velocity
        self.input_Wb = self.traj_W + self.kR * self.Wedge(
            -(np.matmul(traj_R.T, self.R) - np.matmul(self.R.T, traj_R)) / 2.0
        )
        """

        # calculate input Body angular velocity

        self.input_Wb = np.array([[ox], [oy], [oz]], dtype=object)
        """
        ten1 = np.array([ox, oy, oz])
        ten2 = ten1.tolist()
        self.input_Wb = ten2
        """
        # calculate nominal Euler angle and Euler angle rate
        self.Euler_nom[1] = np.arctan(
            (self.ref_acc[0] * np.cos(traj_yaw) +
             self.ref_acc[1] * np.sin(traj_yaw))
            / (self.ref_acc[2])
        )
        self.Euler_nom[0] = np.arctan(
            (self.ref_acc[0] * np.sin(traj_yaw) -
             self.ref_acc[1] * np.cos(traj_yaw))
            / np.sqrt(
                (self.ref_acc[2]) ** 2
                + (
                    self.ref_acc[0] * np.cos(traj_yaw)
                    + self.ref_acc[2] * np.sin(traj_yaw)
                )
                ** 2
            )
        )
        self.Euler_nom[2] = traj_yaw

        self.input_Euler_rate = self.BAV2EAR(self.Euler_nom, self.input_Wb)
        self.Euler_rate_nom = self.BAV2EAR(self.Euler_nom, self.traj_W)

    def mellinger_ctrl(self, t):
        self.trajectory.set_clock(t)
        self.trajectory.set_traj()
        self.Position_controller()
        self.Attitude_controller()

    def log_nom(self, log, t):
        log.write_nom(
            t=t,
            input_acc=self.input_acc,
            input_Wb=self.input_Wb,
            P=self.trajectory.traj_pos + self.tmp_pos,
            V=self.trajectory.traj_vel,
            Euler=self.Euler_nom,
            Wb=self.traj_W,
            Euler_rate=self.Euler_rate_nom,
        )
