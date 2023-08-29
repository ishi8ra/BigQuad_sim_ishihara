# 中型・大型ドローン制御シミュレーション

## 実行ファイル

### ./sim_pid.py

- 実行方法
  ```python
      $ python sim_pid.py
  ```

PIDコントローラを利用したドローンの位置制御シミュレーション <br>
目標位置[0.0, 0.0, 1.0]に収束するようすをターミナルに表示

### ./sample_pid.py

- 実行方法
  ```python
      $ python sample_pid.py
  ```

自身で編集したsample_controllerを利用したドローンの制御シミュレーション


## その他ライブラリ

### ./controller

- pid_controller.py
  - デフォルトの目標位置は[x, y, z]:[0.0, 0.0, 1.0]
  - カスケード型PIDコントローラ（位置，速度，姿勢）

- sample_controller.py
  - 自分の制御プログラムを記述する用

### ./Drone

- Drone_model.py
  - クアッドロータの物理モデル
  - 各4つのモータのPWMと推力F[N]のマップを持つ
  - 入力は各4つのモータのPWM値

- Inner_controller.py
  - 本来crazyflie に書き込む内部コントローラを模したもの
  - 角速度PIDコントローラ
  - 推力F[N]，角速度$\Omega$[rad/s]を受け取り各4つのモータに入力するPWM値に変換する
  - 出力は各4つのモータのPWM値

### ./tools

- Decorator.py
  - デコレータを定義

- Log
  - 目標状態と実状態を記録するためのライブラリ

- Mathfunction.py
  - コントローラで利用する関数群

- pid.py
  - PIDコントローラ
