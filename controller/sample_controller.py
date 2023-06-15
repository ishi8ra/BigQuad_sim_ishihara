import numpy as np
from tools.Mathfunction import Mathfunction as MF


class Controller:
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
        """
        ドローンへの入力を計算
        """
        return self.input_acc, self.input_Wb
