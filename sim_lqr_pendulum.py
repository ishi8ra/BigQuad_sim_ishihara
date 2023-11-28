import numpy as np
from Drone.Drone_model import Drone
from controller.lq_controller import Lq_Controller
from datetime import datetime
import pandas as pd


# csvファイル吐き出し用の設定
current_time = datetime.now().strftime("%m%d_%H%M%S")

data_state_Euler = {
    'Time': [],
    'P_x': [], 'P_y': [], 'P_z': [],
    'Roll': [], 'Pitch': [], 'Yaw': []
}
data_input = {
    'Time': [],
    'input_Zacc': [],
    'input_Wx': [],
    'input_Wy': []
}

# シミュレーションステップの間隔[s]
DT = 0.001
# シミュレーションの終了時間[s]
T = 20
# ドローンの初期状態
P = np.array([1.0, 0.5, 0.0])
V = np.array([0.0, 0.0, 0.0])
R = np.eye(3)
EULER = np.array([0.0, 0.0, 0.0])
W = np.array([0.0, 0.0, 0.0])
EULER_RATE = np.array([0.0, 0.0, 0.0])

# 制御対象のドローンを初期化
BigQuad_1 = Drone(dt=DT)
BigQuad_1.set_initial_state(P, V, R, EULER, W, EULER_RATE, DT)

# コントローラを初期化
lq_controller = Lq_Controller(dt=DT)
lq_controller.set_reference()
# シミュレーション時間[s]
t = 0


excel_path = 'excel_exported/' + 'QR' + '_T=' + \
    str(T) + '_' + current_time + '.xlsx'

while t <= T:
    # コントローラの計算, ドローンの状態を渡す
    lq_controller.set_state(BigQuad_1)
    inputs = lq_controller.controller()
    input_Zacc, input_wb = lq_controller.controller()

    # ドローンへの入力
    BigQuad_1.main(input_Zacc, input_wb)

    # 時間を進める
    t += DT

    # StateをExcelに書き込み
    data_state_Euler['Time'].append(t)
    data_state_Euler['P_x'].append(BigQuad_1.P.now[0])
    data_state_Euler['P_y'].append(BigQuad_1.P.now[1])
    data_state_Euler['P_z'].append(BigQuad_1.P.now[2])
    data_state_Euler['Roll'].append(BigQuad_1.Euler.now[0])
    data_state_Euler['Pitch'].append(BigQuad_1.Euler.now[1])
    data_state_Euler['Yaw'].append(BigQuad_1.Euler.now[2])

    # InputをExcelに書き込み
    data_input['Time'].append(t)
    data_input['input_Zacc'].append(input_Zacc)
    data_input['input_Wx'].append(input_wb[0])
    data_input['input_Wy'].append(input_wb[1])

df_general = pd.DataFrame(data_state_Euler)
df_general.to_excel('excel_exported/' + 'general' + '_T=' + str(T) + '_' +
                    current_time + '.xlsx', index=False)
df_input = pd.DataFrame(data_input)
df_input.to_excel('excel_exported/' +
                  'input_T=' + str(T) + '_' + current_time + '.xlsx', index=False)
