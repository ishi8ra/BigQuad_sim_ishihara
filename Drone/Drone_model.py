from Drone.Inner_controller import Controller_attituede_rate
from Drone.State import State
from tools.Mathfunction import Mathfunction
import numpy as np
import sys

sys.path.append("../")


class Drone(Mathfunction):
    def __init__(self, dt):
        print("Initial DRONE model")

        self.set_parametor(dt)
        self.inner_controller = Controller_attituede_rate(dt, self.mQ, self.I)

    def set_parametor(self, dt):
        self.dt = dt

        # * set physical parametor
        self.g = 9.8
        self.mQ = 0.558
        # self.mQ = 0.424
        self.I = np.array(
            [[10 ** (-1), 0.0, 0.0], [0.0, 10 ** (-1), 0.0],
             [0.0, 0.0, 10 ** (-1)]]
        )
        self.Arm_length = 0.15  # [cm] # 大型
        # self.Arm_length = 0.19 # [cm] 中型
        self.Hegiht = 0.05
        self.e3 = np.array([0, 0, 1.0])

        # * set parametor to destributre motor from input command
        self.CM_MP2FM = np.array(
            [
                [1.0, 1.0, 1.0, 1.0],
                [-1.0, -1.0, 1.0, 1.0],
                [-1.0, 1.0, 1.0, -1.0],
                [1.0, -1.0, 1.0, -1.0],
            ]
        )
        self.M = np.zeros(4)

        # * parametor to plot quadtoror
        self.body_frame = np.array(
            [
                (self.Arm_length, 0, 0, 1),
                (0, self.Arm_length, 0, 1),
                (-self.Arm_length, 0, 0, 1),
                (0, -self.Arm_length, 0, 1),
                (0, 0, 0, 1),
                (0, 0, self.Hegiht, 1),
            ]
        )

    def set_initial_state(self, P, V, R, Euler, Wb, Euler_rate, dt):
        self.P = State(dt, P)
        self.V = State(dt, V)
        self.R = State(dt, R)
        self.Euler = State(dt, Euler)
        self.Wb = State(dt, Wb)
        self.Euler_rate = State(dt, Euler_rate)
        self.A = np.array([0.0, 0.0, 0.0])

    # ! integrate quadrotor states
    def update_state(self, acc, Omega_acc):
        self.A = acc
        self.V.integration(acc)
        self.Wb.integration(Omega_acc)
        self.Euler_rate.update((self.BAV2EAR(self.Euler.now, self.Wb.now)))
        self.P.integration(self.V.pre)
        self.Euler.integration(self.Euler_rate.pre)
        self.R.integration(np.matmul(self.R.now, self.Vee(self.Wb.pre)))

        wHb = np.r_[np.c_[self.R.now, self.P.now], np.array([[0, 0, 0, 1]])]
        self.quadWorldFrame = wHb.dot(self.body_frame.T)
        self.world_frame = self.quadWorldFrame[0:3]

        self.inner_controller.set_state(self.Wb.now, self.Euler_rate.now)

    # ! convert Motor power to Force and Moment
    def MP2FM(self, Moter_Power):
        return np.matmul(self.CM_MP2FM, Moter_Power)

    # ! calculate input PWM from inner controller
    def get_input_acc_and_Wb(self, acc, Wb):
        self.inner_controller.inner_controller_Body(acc, Wb)
        return self.inner_controller.MP_pwm

    # ! satureate each motor power
    def Power_destribution_stock(self, IN_Moter_Power):
        Moter_Power = np.zeros(4)

        Moter_Power[0] = self.saturation(IN_Moter_Power[0], 35000.0, 0.0)
        Moter_Power[1] = self.saturation(IN_Moter_Power[1], 35000.0, 0.0)
        Moter_Power[2] = self.saturation(IN_Moter_Power[2], 35000.0, 0.0)
        Moter_Power[3] = self.saturation(IN_Moter_Power[3], 35000.0, 0.0)

        return Moter_Power

    # ! calculate output acceleration for drone motion from input
    def Drone_Dynamics(self, F, M):
        acc = F * np.matmul(self.R.now, self.e3) / (self.mQ) - self.g * self.e3
        Omega_acc = np.matmul(
            np.linalg.inv(self.I),
            (M - np.cross(self.Wb.now, np.matmul(self.I, self.Wb.now))),
        )

        return acc, Omega_acc

    # ! calculate pwn motor power from Froce and Torque [Newton]
    def MM_pwm2N(self, moter):
        In_m1 = moter[0]
        In_m2 = moter[1]
        In_m3 = moter[2]
        In_m4 = moter[3]

        Out_m = np.zeros(4)

        # * coefficient of pwm-F 2D-curve (Force and Torque)
        m1_map = np.array(
            [2.1866e-09, 2.5864e-05, -0.0699]
        )  # np.array([2.077e-07, 0.0021, 0.0])
        m2_map = np.array(
            [1.9461e-09, 2.5622e-05, -0.0648]
        )  # np.array([2.1910e-07, 0.0022, 0.0])
        m3_map = np.array(
            [2.0772e-09, 2.3301e-05, -0.0495]
        )  # np.array([2.1161e-07, 0.0024, 0.0])
        m4_map = np.array(
            [1.8948e-09, 3.1570e-05, -0.0759]
        )  # np.array([2.0210e-07, 0.0024, 0.0])

        m1_map_torque = np.array([2.5150e-11, 3.9841e-07, -8.5201e-04])
        m2_map_torque = np.array([2.0893e-11, 3.9322e-07, -7.8045e-04])
        m3_map_torque = np.array([2.8182e-11, 3.5346e-07, -7.2414e-04])
        m4_map_torque = np.array([2.1372e-11, 4.9849e-07, -0.0011])

        sign_m1 = np.sign(In_m1)
        Out_m[0] = sign_m1 * np.dot(
            m1_map, np.array([In_m1**2, sign_m1 * np.abs(In_m1), 1.0])
        ) + sign_m1 * np.dot(
            m1_map_torque, np.array([In_m1**2, sign_m1 * np.abs(In_m1), 1.0])
        )

        sign_m2 = np.sign(In_m2)
        Out_m[1] = sign_m2 * np.dot(
            m2_map, np.array([In_m2**2, sign_m2 * np.abs(In_m2), 1.0])
        ) + sign_m2 * np.dot(
            m2_map_torque, np.array([In_m2**2, sign_m2 * np.abs(In_m2), 1.0])
        )

        sign_m3 = np.sign(In_m3)
        Out_m[2] = sign_m3 * np.dot(
            m3_map, np.array([In_m3**2, sign_m3 * np.abs(In_m3), 1.0])
        ) + sign_m3 * np.dot(
            m3_map_torque, np.array([In_m3**2, sign_m3 * np.abs(In_m3), 1.0])
        )

        sign_m4 = np.sign(In_m4)
        Out_m[3] = sign_m4 * np.dot(
            m4_map, np.array([In_m4**2, sign_m4 * np.abs(In_m4), 1.0])
        ) + sign_m4 * np.dot(
            m4_map_torque, np.array([In_m4**2, sign_m4 * np.abs(In_m4), 1.0])
        )

        return Out_m

    # ! execute drone motion flow
    def main(self, acc, Wb):
        self.M = self.get_input_acc_and_Wb(acc, Wb)
        M_pwm = self.Power_destribution_stock(self.M)
        M_gf = self.MM_pwm2N(M_pwm)
        self.F_and_M = self.MP2FM(M_gf)
        acc, Wb_acc = self.Drone_Dynamics(self.F_and_M[0], self.F_and_M[1:])
        self.update_state(acc, Wb_acc)

        return acc, Wb_acc
