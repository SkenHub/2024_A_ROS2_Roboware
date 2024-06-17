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
- `/cmd_vel` (`std_msgs/Float32MultiArray`)：`controller_node`が計算した移動指示（速度、方向、角度）をパブリッシュ。
- `/robot_position` (`std_msgs/Float32MultiArray`)：マイコンからのロボットの自己位置（x, y, θ）。例：`[1500.0, 1500.0, 0.0]`。
- `/realsense_position` (`std_msgs/Float32MultiArray`)：リアルセンスのロボットの自己位置（x, y, θ）。例：`[1500.0, 1500.0, 0.0]`。
- `/estimated_position` (`std_msgs/Float32MultiArray`)：推定したロボットの自己位置（x, y, θ）。例：`[2000.0, 1800.0, 0.0]`。

### メッセージの内容
- `/web_socket_pub`：JSON形式の文字列。例：`{"point": "1"}`。
- `/cmd_vel`：速度（0-100）、方向（0-359度）、角度（0-359度）。例：`[100, 45, 90]`。
- `/robot_position`：マイコンからのロボットの自己位置（x, y, θ）。例：`[1500.0, 1500.0, 0.0]`。
- `/realsense_position`：リアルセンスのロボットの自己位置（x, y, θ）。例：`[1500.0, 1500.0, 0.0]`。
- `/estimated_position`：推定したロボットの自己位置（x, y, θ）。例：`[2000.0, 1800.0, 0.0]`。

## ノードの役割
### `web_socket_node`
- WebSocket通信を介してコントローラーからの指示を受け取る。
- 受け取った指示を`/web_socket_pub`トピックにパブリッシュ。

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

## ライセンス
Apache License 2.0
