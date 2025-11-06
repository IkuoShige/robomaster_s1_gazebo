# RoboMaster S1 Gazebo Simulation

RoboMaster S1ロボットのGazeboシミュレーションパッケージです。メカナムホイールによる全方向移動制御に対応しています。

## 特徴

- **メカナムホイール制御**: 4輪メカナムホイールによる全方向移動（前後・横・回転・複合動作）
- **Livox MID-360 LiDAR**: 3D点群センサーのシミュレーション
- **ジンバル制御**: カメラマウント付きジンバル
- **ROS2 Humble対応**: 最新のROS2環境で動作

## クイックスタート

### 環境セットアップ

```bash
source /home/ikuo/docker_play/s1_ws/src/ros2_ws/install/setup.bash
```

### Gazebo起動

```bash
# 標準起動（ISCAS Museumワールド + LiDAR付き）
ros2 launch robomaster_s1_gazebo s1_gazebo.launch.py
```

### ロボット制御

別ターミナルで以下のコマンドを実行：

```bash
# 前進
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

# 横移動（メカナム特有）
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {z: 0.0}}"

# 回転
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.5}}"
```

### キーボード操作

```bash
ros2 launch robomaster_s1_gazebo s1_teleop_keyboard.launch.py
```

操作方法は起動時に表示されます。

## ROS2インターフェース

### 入力トピック

| トピック | 型 | 説明 |
|---------|-----|------|
| `/cmd_vel` | `geometry_msgs/Twist` | ロボット速度指令（線速度・角速度） |

### 出力トピック

| トピック | 型 | 説明 |
|---------|-----|------|
| `/odom` | `nav_msgs/Odometry` | オドメトリ情報（位置・速度） |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | LiDAR点群データ |
| `/camera/image_raw` | `sensor_msgs/Image` | カメラ画像 |

### TFフレーム

```
odom → base_link → [chassis, wheels, gimbal, sensors]
```

## アーキテクチャ

### パッケージ構成

```
robomaster_s1_gazebo/
├── launch/          # 起動ファイル
│   ├── s1_gazebo.launch.py              # 標準起動（GUI + LiDAR）
│   ├── s1_navigation_gazebo.launch.py   # Navigation統合起動
│   └── s1_teleop_keyboard.launch.py     # キーボード操作
├── worlds/          # Gazeboワールドファイル
│   └── iscas_museum.world
├── urdf/            # ロボットURDFファイル
│   └── robomaster_s1_with_lidar.urdf.xacro
├── scripts/         # ユーティリティスクリプト
│   ├── mecanum_teleop_keyboard.py
│   └── cleanup_gazebo.sh
```

## 依存パッケージ

### システム依存

- ROS2 Humble
- Gazebo 11

### ROS2パッケージ

```bash
# ビルド依存
ament_cmake

# 実行時依存
gazebo_ros
robot_state_publisher
robomaster_description
ros2_livox_simulation
```

## ビルド

```bash
cd /home/ikuo/docker_play/s1_ws/src/ros2_ws
colcon build --packages-select robomaster_s1_gazebo
source install/setup.bash
```

## トラブルシューティング

### Gazeboが起動しない

```bash
# 古いプロセスをクリーンアップ
pkill -9 -f "gazebo|gzserver|gzclient"

# 再起動
ros2 launch robomaster_s1_gazebo s1_gazebo.launch.py
```

### ロボットが動かない

1. トピックの確認
   ```bash
   ros2 topic list | grep -E "cmd_vel|odom"
   ```

2. プラグインの確認
   ```bash
   ros2 node list | grep planar_move
   ```

## ライセンス

このソフトウェアパッケージは，Apache 2.0 ライセンスの下，再頒布および使用が許可されます．


