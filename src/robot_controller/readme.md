# robot_controller

## 概要
`robot_controller`は、ロボットの移動を制御するためのROS2パッケージです。このパッケージは、WebSocket経由でコントローラーから指示を受け取り、指定された地点にロボットを移動させます。また、ロボットの自己位置をマイコンとリアルセンスから受信し、ROS2トピックに公開します。

## システム構成
### ノード構成
- `web_socket_node`：WebSocket通信を介してコントローラーから指示を受け取り、ROS2トピックにパブリッシュします。
- `controller_node`：WebSocketノードからの指示を受け取り、ロボットを指定された地点に移動させるための速度、方向、および角度を計算し、シリアル通信ノードに送信します。速度は台形プロファイルに基づいてスムーズに増加します。
- `cmd_vel_to_serial_node`：`controller_node`からの移動指示を受け取り、シリアル通信を介してマイコンに送信します。
- `serial_to_position_node`：マイコンから自己位置データを受信し、ROS2トピックにパブリッシュします。
- `realsense_position_node`：RealSense T265から自己位置データを取得し、マイコンからのデータと組み合わせて推定位置をパブリッシュします。

## 通信の概要
### トピック
- `/web_socket_pub` (`std_msgs/String`)：WebSocketノードからの指示をパブリッシュ。
- `/cmd_vel` (`std_msgs/Float32MultiArray`)：`controller_node`が計算した移動指示（速度、方向、角度、モード、動作番号）をパブリッシュ。
- `/robot_position` (`std_msgs/Float32MultiArray`)：マイコンからのロボットの自己位置（x, y, θ）。例：`[1500.0, 1500.0, 0.0]`。
- `/realsense_position` (`std_msgs/Float32MultiArray`)：リアルセンスのロボットの自己位置（x, y, θ）。例：`[1500.0, 1500.0, 0.0]`。
- `/estimated_position` (`std_msgs/Float32MultiArray`)：推定したロボットの自己位置（x, y, θ）。例：`[2000.0, 1800.0, 0.0]`。

### メッセージの内容
- `/web_socket_pub`：文字列形式のデータ。例：`"1,0,0,0,1,1,0,0,1,0"`。
  データ内容
　データの内容は動作1~4,目標位置1~3,チームカラー,遠隔非常停止(プログラム上)
　動作1~5,動作1~3,遠隔非常停止は0,1が送られてくる　チームカラーは0:青,1:赤で送られてくる
　データは,を挟んで送られてくる　"動作1,動作2,動作3,動作4,動作5,目標位置1,目標位置2,目標位置3,チームカラー,遠隔非常停止"
- `/cmd_vel`：速度（m/s）、方向（0-359度）、角度（0-359度）、モード（0または1）、動作番号。例：`[1.0, 45, 90, 1, 2]`。
- `/robot_position`：マイコンからのロボットの自己位置（x, y, θ）。例：`[1500.0, 1500.0, 0.0]`。
- `/realsense_position`：リアルセンスのロボットの自己位置（x, y, θ）。例：`[1500.0, 1500.0, 0.0]`。
- `/estimated_position`：推定したロボットの自己位置（x, y, θ）。例：`[2000.0, 1800.0, 0.0]`。

## マイコンとの通信
### 送信データ形式 (PCからマイコンへ)
- 速度（m/s）: float (4バイト)
- 方向（度）: float (4バイト)
- 角度（度）: float (4バイト)
- モード（赤1または青0）: uint8 (1バイト)
- 動作番号: uint8 (1バイト)

合計で 14 バイトのデータが送信されます。

### 受信データ形式 (マイコンからPCへ)
- ロボットの自己位置（x[mm], y[mm], θ[deg]）: float (各4バイト、合計12バイト)
- モード（赤1または青0）: uint8 (1バイト)
- 動作番号: uint8 (1バイト)

合計で 14 バイトのデータが受信されます。

## ノードの役割
### `web_socket_node`
- WebSocket通信を介してコントローラーからの指示を受け取る。
- 受け取った指示を`/web_socket_pub`トピックにパブリッシュ。
- fastapi、uvicorn をインストールする必要がある。
```bash
pip install fastapi
pip install uvicorn
```
- R1_UI.txtの294行目のipアドレスとポートを実際に使用する番号に変える必要がある。
```bash
const ws = new WebSocket("ws://ipアドレス:ポート/ws");
```
- web_socket_node.pyの13,14行目も実際に使用する番号に変える必要がある。
```bash
ipadress_ = 'ipアドレス'
port_ = ポート
```

### `controller_node`
- `web_socket_node`からの指示を受け取り、指定された地点に移動するための速度、方向、および角度を計算。
- 速度は台形プロファイルに基づいてスムーズに増加。
- 計算結果を`/cmd_vel`トピックにパブリッシュ。

### `cmd_vel_to_serial_node`
- `/cmd_vel`トピックから移動指示を受け取り、シリアル通信でマイコンに送信。

### `serial_to_position_node`
- マイコンから自己位置データをシリアル通信で受信。
- 受信したデータを`/robot_position`トピックにパブリッシュ。

### `realsense_position_node`
- RealSense T265から自己位置データを取得。
- マイコンからの自己位置データと組み合わせて推定位置を計算。
- 推定位置を`/estimated_position`トピックにパブリッシュ。

## 注意事項
- シリアルポートの設定や接続には注意が必要です。正しいポートを指定してください。
- WebSocketのIPアドレスとポートは環境に応じて変更してください。

## 実行方法
### 環境設定

1. **ROS2 Humbleのソース**
    ```bash
    source /opt/ros/humble/setup.bash
    source ~/ros2_robocon_ws/2024_A_ROS2_Roboware/install/setup.bash
    ```

2. **ROS2ワークスペースのビルド**
    ```bash
    cd ~/ros2_robocon_ws/2024_A_ROS2_Roboware
    colcon build
    source install/setup.bash
    ```

### ノードの実行コマンド

各ノードを個別のターミナルウィンドウで実行します。

1. **`web_socket_node` の実行**
    ```bash
    ros2 run robot_controller web_socket_node
    ```

2. **`controller_node` の実行**
    ```bash
    ros2 run robot_controller controller_node
    ```

3. **`cmd_vel_to_serial_node` の実行**
    ```bash
    ros2 run robot_controller cmd_vel_to_serial_node
    ```

4. **`serial_to_position_node` の実行**
    ```bash
    ros2 run robot_controller serial_to_position_node
    ```

5. **`realsense_position_node` の実行**
    ```bash
    ros2 run robot_controller realsense_position_node
    ```

### 確認手順
1. 各ノードを個別のターミナルで起動します。
2. `web_socket_node` が起動したら、Webブラウザを開き、`http://192.168.0.175:8000` にアクセスしてUIを確認します。


## ライセンス
Apache License 2.0
