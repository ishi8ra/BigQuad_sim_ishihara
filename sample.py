import numpy as np
from Drone.Drone_model import Drone
from controller.sample_controller import Controller

# シミュレーションステップの間隔[s]
DT = 0.001
# シミュレーションの終了時間[s]
T = 10
# ドローンの初期状態
P = np.array([0.0, 0.0, 0.0])
V = np.array([0.0, 0.0, 0.0])
R = np.eye(3)
EULER = np.array([0.0, 0.0, 0.0])
W = np.array([0.0, 0.0, 0.0])
EULER_RATE = np.array([0.0, 0.0, 0.0])

# 制御対象のドローンを初期化
BigQuad_1 = Drone(dt=DT)
BigQuad_1.set_initial_state(P, V, R, EULER, W, EULER_RATE, DT)

# コントローラを初期化
sample_controller = Controller()

# シミュレーション時間[s]
t = 0

while t <= T:
    # コントローラの計算, ドローンの状態を渡す
    input_acc, input_wb = sample_controller(BigQuad_1)

    # ドローンへの入力
    BigQuad_1.main(input_acc, input_wb)

    # 時間を進める
    t += DT
