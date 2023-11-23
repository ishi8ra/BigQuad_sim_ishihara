from scipy import signal
import numpy as np
from numpy import sin, cos
import matplotlib.pyplot as plt

# パラメータ
r = 2 / 3
g = 9.8  # 重力加速度 [m/s^2]

# システム行列
A = np.array([[0, 1, 0, 0, 0, 0, 0, 0],
              [0, 0, g, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0],
              [0, 0, 0, 0, 0, -g/r, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 1],
              [0, 0, 0, 0, 0, 0, 0, 0]])

# 入力行列
Bl = np.array([[0, 0, 0],
               [0, 0, 0],
               [1, 0, 0],
               [0, 0, 0],
               [0, 0, 0],
               [0, 1, 0],
               [0, 0, 0],
               [0, 0, 1]])

# 重み行列
Q = np.diag([1, 1, 1, 1, 1, 1, 1, 1])
R = np.diag([1, 1, 1])

# scipyのcontrolライブラリを使用してLQRゲインを計算
# K, S, E = signal.lqr(A, Bl, Q, R)

K = np.array([
    [1.00000000000000, 1.45175395145263, 5.42718872423573, 8.97157015039534e-17,
     2.36934578759692e-16, -1.51758575511169e-15, -1.83826241986066e-16, 3.25349858845081e-16],
    [-4.54758256147348e-16, -2.68464342807294e-16, 1.87287620071113e-16, -0.999999999999999, -
     1.36885555678166, 6.42217668469037, 1.48180184268838e-15, 3.30898740618047e-16],
    [1.79859175550358e-16, 4.09738600794711e-16, -1.25760985716651e-16, 5.83691553045932e-17,
     1.23204693030713e-16, -2.31044461374617e-16, 1.00000000000000, 1.73205080756888]
])

# シミュレーションの設定
Tf = 30  # シミュレーション時間 [s]
Ts = 0.001  # サンプリング時間 [s]
T = np.arange(0, Tf + Ts, Ts)  # シミュレーションの時間軸
N = len(T)  # シミュレーションの時間ステップ数

# 目標値の定義
q_ref = np.array([[0], [0], [0], [0], [0], [0], [1], [0]])

# 初期状態
q0 = np.zeros((8, 1))


u = np.zeros((3, N))
q = np.zeros((8, N))
dq = np.zeros((8, N))
q[:, 0] = q0.squeeze()

# シミュレーションの実行
for k in range(N - 1):
    # 状態フィードバックによる制御入力の計算
    u[:, k] = -K @ (q[:, k] - q_ref.squeeze())

    # 状態方程式の計算
    dq[:, k] = np.array([q[1, k],
                         (g+0.5*g*(q[2, k]*q[2, k]+q[5, k]
                                   * q[5, k])+u[2, k])*sin(q[2, k]),
                         u[0, k],
                         q[4, k],
                         -(g+0.5*g*(q[2, k]*q[2, k]+q[5, k]*q[5, k])+u[2, k]) *
                         sin(q[5, k])*cos(q[2, k]),  # 5
                         u[1, k]/cos(q[2, k]),
                         q[7, k],
                         (g+0.5*g*(q[2, k]*q[2, k]+q[5, k]*q[5, k])+u[2, k])*cos(q[2, k])*cos(q[5, k])-g])
    # 1ステップ先の状態の更新
    q[:, k+1] = q[:, k] + dq[:, k]*Ts

plt.figure()
plt.plot(T, q[0, :], label='x')
plt.plot(T, q[3, :], label='y')
plt.plot(T, q[6, :], label='z')
plt.plot(T, q[2, :], label='β')
plt.plot(T, q[5, :], label='γ')
plt.legend()
plt.xlabel('Time [s]')
plt.ylabel('State')
plt.grid(True)
plt.show()

plt.figure()
plt.plot(T, u[0, :], label='wx')
plt.plot(T, u[1, :], label='wy')
plt.plot(T, u[2, :], label='ax')
plt.legend()
plt.xlabel('Time [s]')
plt.ylabel('Input')
plt.grid(True)
plt.show()
