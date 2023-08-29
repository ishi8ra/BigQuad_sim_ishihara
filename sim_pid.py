import numpy as np
from Drone.Drone_model import Drone
from controller.pid_controller import Pid_Controller

# シミュレーションステップの間隔[s]
DT = 0.001
# シミュレーションの終了時間[s]
T = 20
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

# コントローラを初期化,目標値を設定
pid_controller = Pid_Controller(dt=DT)
pid_controller.set_reference()
# シミュレーション時間[s]
t = 0

while t <= T:
    # コントローラの計算, ドローンの状態を渡す
    pid_controller.set_state(BigQuad_1)
    # ドローンへの入力を計算する
    input_acc, input_wb = pid_controller.controller()

    # ドローンへの入力
    BigQuad_1.main(input_acc, input_wb)

    # 時間を進める
    t += DT
    
    # 位置情報を表示
    print(BigQuad_1.P.now)
