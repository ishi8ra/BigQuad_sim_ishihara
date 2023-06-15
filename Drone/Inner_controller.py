import sys

sys.path.append("../")

import numpy as np
from tools.pid import PID
from tools.Mathfunction import Mathfunction


class Controller_attituede_rate(Mathfunction):
    def __init__(self, dt, mQ, I):
        # * initialize model parametor
        self.mQ = mQ
        self.I = I
        self.g = 9.8

        # * initialize attitude rate pid conrtoller (Euler and Body angular velocity)
        # Euler angle velocity PID
        self.R_rate_pid = PID(5.0, 2, 0.0, dt, 1.5)
        self.P_rate_pid = PID(5.0, 2, 0.0, dt, 1.5)
        self.Y_rate_pid = PID(15.0, 5, 0.0, dt, 1.5)

        # Body angle velocity PID
        self.Wx_pid = PID(2.0, 0, 0.0, dt, 1.5)
        self.Wy_pid = PID(2.0, 0, 0.0, dt, 1.5)
        self.Wz_pid = PID(0.5, 0, 0.0, dt)

        self.M_gf = np.array([0.0, 0.0, 0.0])
        self.FM_pwm = np.array([0.0, 0.0, 0.0, 0.0])

        self.MP_gf = np.array([0.0, 0.0, 0.0, 0.0])
        self.MP_pwm = np.array([0.0, 0.0, 0.0, 0.0])

        self.rad2deg = 180.0 / np.pi
        self.deg2rad = np.pi / 180.0

        # * control matrix
        KT = self.mQ / 4
        KR = np.sqrt(2) * I[1, 1] / 0.15 / 4
        KP = np.sqrt(2) * I[0, 0] / 0.15 / 4
        KY = I[2, 2] / 4
        self.FM2MP_map = np.array(
            [
                [KT, KT, KT, KT],
                [-KR, -KR, KR, KR],
                [-KP, KP, KP, -KP],
                [-KY, KY, -KY, KY],
            ]
        ).T
        self.MP2FM_map = np.array(
            [
                [1.0, 1.0, 1.0, 1.0],
                [-1.0, -1.0, 1.0, 1.0],
                [-1.0, 1.0, 1.0, -1.0],
                [-1.0, 1.0, -1.0, 1.0],
            ]
        )

    def set_state(self, Wb, Euler_rate):
        self.Wb = Wb
        self.Euler_rate = Euler_rate

        self.Wx_pid.state = Wb[0]
        self.Wy_pid.state = Wb[1]
        self.Wz_pid.state = Wb[2]

        self.R_rate_pid.state = Euler_rate[0] * self.rad2deg
        self.P_rate_pid.state = -Euler_rate[1] * self.rad2deg
        self.Y_rate_pid.state = Euler_rate[2] * self.rad2deg

    # ! use Euler angle rate pid
    def inner_controller_Euler(self, F, Euler_rate_input):
        self.R_rate_pid.desired = Euler_rate_input[0]
        self.P_rate_pid.desired = Euler_rate_input[1]
        self.Y_rate_pid.desired = Euler_rate_input[2]

        self.controll_attitude_rate_Euler()
        self.FM2MP(F)
        self.MM_gf2pwm()
        self.MM_gf2pwm2(F)

    def controll_attitude_rate_Euler(self):
        self.R_rate_pid.runpid()
        self.P_rate_pid.runpid()
        self.Y_rate_pid.runpid()

        self.M_gf[0] = self.R_rate_pid.output * self.deg2rad
        self.M_gf[1] = self.P_rate_pid.output * self.deg2rad
        self.M_gf[2] = self.Y_rate_pid.output * self.deg2rad

    # ! use body angle velocity pid
    def inner_controller_Body(self, acc, Wb):
        self.Wx_pid.desired = Wb[0]
        self.Wy_pid.desired = Wb[1]
        self.Wz_pid.desired = Wb[2]

        self.controll_attitude_rate_body()
        self.FM2MP(acc)
        self.MM_gf2pwm()

    def controll_attitude_rate_body(self):
        self.Wx_pid.runpid()
        self.Wy_pid.runpid()
        self.Wz_pid.runpid()

        self.M_gf[0] = self.Wx_pid.output
        self.M_gf[1] = self.Wy_pid.output
        self.M_gf[2] = self.Wz_pid.output

    # ! convert Force and Moment into Motor Power [PWM]
    def FM2MP(self, F):
        self.MP_gf_TRP = np.matmul(
            self.FM2MP_map, np.array([F, self.M_gf[0], self.M_gf[1], 0.0])
        )  # Thrust, Roll, Pitch
        self.MP_gf_Y = np.matmul(
            self.FM2MP_map, np.array([0.0, 0.0, 0.0, self.M_gf[2]])
        )  # Yaw

    # ! convert Motor Poewr Force into PWM
    def MM_gf2pwm(self):
        # ToDo モータの推力とトルクのマップ関数を作る　f1_thrust f2_thrust f3_thrust f4_thrust f1_torpue f2_torpue f3_torpue f4_torpue

        In_m1 = self.MP_gf_TRP[0]
        In_m2 = self.MP_gf_TRP[1]
        In_m3 = self.MP_gf_TRP[2]
        In_m4 = self.MP_gf_TRP[3]

        sign_m1 = np.sign(In_m1)
        self.MP_pwm[0] = sign_m1 * (
            np.sqrt(sign_m1 * In_m1 * 8.746454211812262e-9 + 1.280608521387491e-9)
            * 2.28664090792239e8
            - 5.914182129481692e3
        )
        # self.MP_pwm[0] = sign_m1 * (-1.0785e-5 * In_m1**4 + sign_m1* 0.0080 * In_m1**3 - 2.0815 * In_m1**2 +  306.7172*In_m1 + 1.7019e+03)

        sign_m2 = np.sign(In_m2)
        self.MP_pwm[1] = sign_m2 * (
            np.sqrt(sign_m2 * In_m2 * 7.784387419895479e-9 + 1.160848849564822e-9)
            * 2.56924519826488e8
            - 6.582814614736632e3
        )
        # self.MP_pwm[1] = sign_m2 * (-8.3705e-06 * In_m2**4 + sign_m2* 0.0066 * In_m2**3 - 1.8432 * In_m2**2 +  288.9068*In_m2 + 1.8032e+03)

        sign_m3 = np.sign(In_m3)
        self.MP_pwm[2] = sign_m3 * (
            np.sqrt(sign_m3 * In_m3 * 8.308971069381441e-9 + 9.542541377640133e-10)
            * 2.407036904208272e8
            - 5.608689869160311e3
        )
        # self.MP_pwm[2] = sign_m3 * (-8.6533e-06 * In_m3**4 + sign_m3*  0.0068 * In_m3**3 -1.8811 * In_m3**2 + 290.2357*In_m3 + 1.8179e+03)

        sign_m4 = np.sign(In_m4)
        self.MP_pwm[3] = sign_m4 * (
            np.sqrt(sign_m4 * In_m4 * 7.579233605794426e-9 + 1.571593599291622e-9)
            * 2.638789228598223e8
            - 8.330685304890129e3
        )
        # self.MP_pwm[3] = sign_m4 * (-9.6405e-06 * In_m4**4 + sign_m4* 0.0073 * In_m4**3 -1.9698 * In_m4**2 +  298.3715*In_m4 + 1.6579e+03)

        In_m1_torque = -self.MP_gf_Y[2]
        In_m2_torque = self.MP_gf_Y[2]
        In_m3_torque = -self.MP_gf_Y[2]
        In_m4_torque = self.MP_gf_Y[2]

        sign_m1 = np.sign(In_m1_torque)
        # poly 4 map
        # self.MP_pwm[0] += sign_m1 * (sign_m1*In_m1_torque*2.141865320609748e+6-In_m1_torque**2.0*1.033497756927838e+8+sign_m1*In_m1_torque**3.0*2.905723618527212e+9-In_m1_torque**4.0*2.884278638991346e+10+1.300532417476346e+3)

        # route map
        self.MP_pwm[0] += sign_m1 * (
            np.sqrt(
                sign_m1 * In_m1_torque * 1.006001536082039e-10 + 1.587299594622495e-13
            )
            * 1.988068534953908e10
            - 7.920649662566478e3
        )

        sign_m2 = np.sign(In_m2_torque)
        # poly 4 map
        # self.MP_pwm[1] += sign_m2 * (sign_m2*In_m2_torque*2.432737424685642e+6-In_m2_torque**2.0*1.286871454413272e+8+sign_m2*In_m2_torque**3.0*3.928300224414117e+9-In_m2_torque**4.0*4.228794888833972e+10+9.727547991878159e+2)

        # route map
        self.MP_pwm[1] += sign_m2 * (
            np.sqrt(
                sign_m2 * In_m2_torque * 9.66189557532568e-11 + 1.932237720338033e-13
            )
            * 2.069987182543716e10
            - 9.09908944642907e3
        )

        sign_m3 = np.sign(In_m3_torque)
        # poly 4 map
        # self.MP_pwm[2] += sign_m3 * (sign_m3*In_m3_torque*2.094467788762548e+6-In_m3_torque**2.0*9.594678104859331e+7+sign_m3*In_m3_torque**3.0*2.521879977781375e+9-In_m3_torque**4.0*2.349805874173285e+10+1.278771640750342e+3)

        # route map
        self.MP_pwm[2] += sign_m3 * (
            np.sqrt(
                sign_m3 * In_m3_torque * 1.12726263933792e-10 + 1.24935566032467e-13
            )
            * 1.774209425741873e10
            - 6.271160652750821e3
        )

        sign_m4 = np.sign(In_m4_torque)
        # poly 4 map
        # self.MP_pwm[3] += sign_m4 * (sign_m4*In_m4_torque*2.084297084480703e+6-In_m4_torque**2.0*1.004366436546806e+8+sign_m4*In_m4_torque**3.0*2.901064905848298e+9-In_m4_torque**4.0*2.949112617497505e+10+1.286587313476963e+3)

        # route map
        self.MP_pwm[3] += sign_m4 * (
            np.sqrt(
                sign_m4 * In_m4_torque * 8.969381778123772e-11 + 1.667909502882046e-13
            )
            * 2.229808084296265e10
            - 9.106546870860521e3
        )
