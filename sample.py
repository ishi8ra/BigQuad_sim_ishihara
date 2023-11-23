import numpy as np
from Drone.Drone_model import Drone
from controller.lqr_controller import Lqr_Controller
import csv
from datetime import datetime


# csvファイル吐き出し用の設定
current_time = datetime.now().strftime("%Y%m%d_%H%M%S")

file_names = {
    'K': 'csv_just_exported/K_' + current_time + '.csv',
    'P': 'csv_just_exported/P_' + current_time + '.csv',
    'V': 'csv_just_exported/V_' + current_time + '.csv',
    'acc': 'csv_just_exported/acc_' + current_time + '.csv',
    'Euler': 'csv_just_exported/Euler_' + current_time + '.csv',
    'W': 'csv_just_exported/W_' + current_time + '.csv',
    'Euler_rate': 'csv_just_exported/Euler_rate_' + current_time + '.csv',
    'input': 'csv_just_exported/input_' + current_time + '.csv',
}

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

# コントローラを初期化
lqr_controller = Lqr_Controller(dt=DT)
lqr_controller.set_reference()
# シミュレーション時間[s]
t = 0

while t <= T:
    # コントローラの計算, ドローンの状態を渡す
    lqr_controller.set_state(BigQuad_1)
    input_acc, input_wb = lqr_controller.controller()

    # ドローンへの入力
    BigQuad_1.main(input_acc, input_wb)

    # 時間を進める
    t += DT

    # 位置情報を表示
    print(BigQuad_1.P.now)

    with open(file_names['P'], 'a') as f:
        writer = csv.writer(f)
        writer.writerow(BigQuad_1.P.now)

    with open(file_names['acc'], 'a') as f:
        writer = csv.writer(f)
        writer.writerow([input_acc])

    with open(file_names['input'], 'a') as f:
        writer = csv.writer(f)
        writer.writerow(lqr_controller.input)
