# FASTLIO2_ROS2 — Hesai JT-128 / ROS 2 Jazzy

Hesai JT-128の点群と内蔵IMUを使い、次の運用を行うためのROS 2 Jazzy対応forkです。

- FAST-LIO2によるLiDAR-inertial odometry
- 3D点群地図の生成・保存
- 保存済みPCD地図に対するICP自己位置推定

このブランチはupstreamの`f516daac08bc46e50e814a2e7d6c8352ed8141bb`を基にしています。ROSパッケージ名は衝突しにくいように`fastlio2`、`fastlio2_interfaces`、`fastlio2_pgo`、`fastlio2_localizer`へ整理しています。HBAは通常運用には含めず、`COLCON_IGNORE`でビルド対象外にしています。

## 入力

CastellaのHesaiドライバが配信する次のトピックを既定で使用します。ドライバ自体はこのリポジトリのlaunchでは起動しません。

| topic | type | frame |
|---|---|---|
| `/lidar_points` | `sensor_msgs/msg/PointCloud2` | `hesai_lidar` |
| `/lidar_imu` | `sensor_msgs/msg/Imu` | `hesai_lidar` |

実機の`/lidar_points`は、`FLOAT32 x/y/z/intensity`、`UINT16 ring`、`FLOAT64 timestamp`を持つ26 byte/pointのPointCloud2です。`timestamp`は非アライン位置にあるため、変換処理は安全なバイトコピーで読み取ります。現在のdual-return設定では約230,400点/scanです。

購読は実機ドライバに合わせてRELIABLE QoSを使用します。この環境で検証した同梱のv2.0.11由来Hesaiドライバは、加速度をm/s²、角速度をrad/sで配信するため、追加の単位変換は行いません。上流の版によって単位変換の仕様が異なるため、ドライバ更新時は静止中の加速度normが約9.8であることと角速度単位を必ず再確認してください。

`/lidar_packets_loss`（`hesai_ros_driver/msg/LossPacket`）はFAST-LIOの入力ではなく、パケット欠落を監視する診断トピックです。

入力を確認するには以下を使用します。

```bash
ros2 topic info --verbose /lidar_points
ros2 topic hz /lidar_points
ros2 topic hz /lidar_imu
ros2 topic echo /lidar_packets_loss --once
```

## ビルド

```bash
cd /home/ytpc2025k/ytlab_ws/ytlab_castella/third_party/FASTLIO2_ROS2
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths \
  fastlio2 fastlio2_interfaces fastlio2_pgo fastlio2_localizer \
  --ignore-src -r -y
colcon build --symlink-install --packages-select \
  fastlio2_interfaces fastlio2 fastlio2_pgo fastlio2_localizer
source install/setup.bash
```

Sophusは通常運用に不要です。GTSAMが見つかる場合はPGOのループ閉じ込みが有効になります。GTSAMがない場合も同じmapping launchでLIO地図の生成・保存は可能ですが、その場合の地図はLIO軌跡の積算で、回環補正は行いません。

Ubuntu 24.04の`libgtsam-dev`は、同梱CMake exportがパッケージに存在しない`libCppUnitLite.a`を参照します。本リポジトリはこのexportを読み込まず、公開ヘッダと`libgtsam.so`を直接検出します。ビルド時に`GTSAM found at ...: pose-graph loop closure is enabled`と表示されることを確認してください。

## LIOのみを起動

Hesaiドライバを先に起動してからLIOを起動し、初期化完了までロボットを静止させます。JT128設定では連続入力を5秒確認してから姿勢を配信します。起動前だけ静止しても初期化sampleには入りません。

```bash
ros2 launch fastlio2 jt128_lio.launch.py
```

主な出力は次のとおりです。

- `/fastlio2/lio_odom`
- `/fastlio2/lio_path`
- `/fastlio2/body_cloud`
- `/fastlio2/world_cloud`
- TF `fastlio_odom -> base_footprint`

RVizを起動しない場合は`enable_rviz:=false`、TFを別ノードが管理する場合は`publish_tf:=false`を指定します。

現在のPCと実トピックでは、既定の高密度設定（scan voxel 0.15 m、map voxel 0.20 m）で約10 HzのLIO出力を確認しています。走行時もCPU負荷と処理周期を監視し、処理落ちする場合はvoxel間隔や最大距離を段階的に調整してください。

## 地図を生成・保存

`jt128_mapping.launch.py`はLIOと地図保存ノードを一緒に起動します。上記のLIO launchと同時には起動しないでください。

```bash
ros2 launch fastlio2_pgo jt128_mapping.launch.py
```

十分に走行して地図を作成したら、ロボットを停止させ、空の保存先を用意してサービスを呼びます。既存ファイルの上書きを防ぐため、保存先が空でない場合は失敗します。

```bash
mkdir -p /home/ytpc2025k/maps/25_2f_fastlio
ros2 service call /fastlio2/pgo/save_maps \
  fastlio2_interfaces/srv/SaveMaps \
  "{file_path: '/home/ytpc2025k/maps/25_2f_fastlio', save_patches: false}"
```

`map.pcd`と`poses.txt`が保存されます。`save_patches: true`では各キーフレームの点群も`patches/`へ保存します。地図保存時には既定で0.10 m voxelへダウンサンプリングします。

保存される3D地図の原点は、LIOを開始して初期化が完了したときの`base_footprint`です。既存の25_2F用2D Nav2地図の`map`原点とは自動では一致しません。このREADMEの手順はFAST-LIO単体の地図生成・自己位置推定までを対象とし、Nav2と同じ地図座標へ統合する場合は、別途`map`間の初期変換または保存後のPCD変換が必要です。

## 事前地図で自己位置推定

地図生成launchを停止してから、保存した`map.pcd`を指定します。地図生成と自己位置推定はどちらもTF `map -> fastlio_odom`を管理するため、同時には起動できません。

```bash
ros2 launch fastlio2_localizer jt128_localization.launch.py \
  map_path:=/home/ytpc2025k/maps/25_2f_fastlio/map.pcd
```

RVizの「2D Pose Estimate」で地図上のおおよその初期位置と向きを指定します。`/initialpose`を受理した後、現在のLIO点群を事前地図へICP整合し、成功すると次を出力します。

- `/fastlio2/localizer/pose`
- `/fastlio2/localizer/map_cloud`
- TF `map -> fastlio_odom`

ICP補正は既定で1 Hzですが、成功後のTFとglobal poseは入力LIOと同じ周期で再配信します。

起動引数で初期姿勢を与えることもできます。角度はradです。

```bash
ros2 launch fastlio2_localizer jt128_localization.launch.py \
  map_path:=/home/ytpc2025k/maps/25_2f_fastlio/map.pcd \
  initialize_from_parameters:=true \
  initial_x:=0.0 initial_y:=0.0 initial_z:=0.0 initial_yaw:=0.0
```

起動後にサービスで再初期化する場合、既に読み込んだ地図を使うときは`pcd_path`を空にできます。

```bash
ros2 service call /fastlio2/localizer/relocalize \
  fastlio2_interfaces/srv/Relocalize \
  "{pcd_path: '', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"

ros2 service call /fastlio2/localizer/relocalize_check \
  fastlio2_interfaces/srv/IsValid "{code: 0}"
```

`relocalize`の成功応答は地図と初期値を受理したことを示します。実際のICP成功は`relocalize_check`の`valid: true`、`/fastlio2/localizer/pose`、RViz上の整合状態で確認してください。整合に成功するまでは`map -> fastlio_odom`を配信しません。

## TFと外部パラメータ

推定器内部の状態はIMU座標系です。JT-128では公開する姿勢・経路・点群を`jt128.yaml`の`r_bl`、`t_bl`で`base_footprint`へ変換します。既定値は現在のCastella URDFにある`base_footprint -> hesai_lidar`（並進`[0.06125, 0.0, 0.728] m`、yaw `-90°`）に対応します。LiDAR取付TFを直接固定するため、オンラインのLiDAR–IMU外部パラメータ推定が進んでも車体点群の基準は変わりません。

LiDARと内蔵IMU間は`r_il=I`、`t_il=0`を初期値とし、`esti_il: true`で推定します。これはHesai公式サンプルで使われる推定開始値であり、実機校正済みの外部パラメータではありません。公式に確認できるのはLiDARと内蔵IMUの軸に回転差がないことまでで、並進は未確定です。高精度運用前には実機データで外部パラメータを校正し、必要に応じて推定値を固定してください。

既定の `nav2_mode:=false` でのTF所有関係は次のとおりです。

```text
map -> fastlio_odom -> base_footprint
```

同じ子frameへ別のTFを同時配信しないでください。CastellaのNSK odometryは通常`odom -> base_footprint`を配信するため、そのままFAST-LIOのTFと併用すると`base_footprint`が二重親になります。Hesaiなどのbringup機能を併用してFAST-LIOをTFの所有者にする場合は、Castella側を`publish_odom_tf:=false`で起動してください。反対に既存システム側をTFの所有者にする場合はFAST-LIO launchへ`publish_tf:=false`を指定します。既定モードでは、Nav2が使用するodom frame/topicの切替は自動実行しません。下記の `nav2_mode` は別途明示して有効化します。

## Nav2向けIMU予測・平面TF（オプション）

```bash
ros2 launch fastlio2_localizer jt128_localization.launch.py \
  map_path:=/absolute/path/map.pcd nav2_mode:=true enable_rviz:=false
```

`nav2_mode:=true` はLIOとICPの直接TF配信を止め、独立した `nav_tf_node` に集約します。
LIOのIMU受信・予測は専用executorで動作し、LiDAR補正後のIMU履歴を再積分します。
現在ROS時刻までの予測を100 Hzで公開し、最後のIMUからの積分は最大50 ms、
最後のLiDAR補正からは最大500 msです。期限超過時は古い姿勢を再配信せず停止します。
この方式の構成は [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/imuPreintegration.cpp#L406-L505)
および [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2/blob/0d2c0346107b75b59934975adec9a6eeeb913c64/src/LIVMapper.cpp#L556-L634)
の補正後IMU再積分を参考にし、状態方程式はこの実装のIESKFと共通化しています。

TFは `map -> odom -> base_footprint`（平面）と
`map -> fastlio_odom -> fastlio_body`（3次元）の2枝です。
このモードではLIO/localizerのframe overrideをこれらの専用フレームに置き換えます。
地図照合は元の3次元の `lio_odom` / `body_cloud` を使います。
平面化は平坦な床のNav2向けで、斜面上の実際の車体傾斜は表現しません。
`nav2_planar:=false` で平面化だけを無効化できます。

平面TFは合成後のmap姿勢を射影し、
`T_map_odom = P(T_map_local * T_local_body) * inverse(P(T_local_body))`
として地図座標の傾きにも対応します。z=0、roll=pitch=0が既定です。
`nav_tf_node.base_height` で固定zを変更できます。
再初期化中は `fastlio2_interfaces/LocalizationState.valid=false` を配信し、
map補正のTF更新を止めます。ICP不収束時は従来どおり最後の成功補正を保持します。

追加出力は `/fastlio2/predicted_odom`、`/fastlio2/nav_odom`、
`/fastlio2/prediction/diagnostics`、`/fastlio2/nav_tf/diagnostics`、
`/fastlio2/localizer/localization_state` です。
`predicted_odom` の共分散は未対応、`nav_odom` は姿勢観測専用です。
制御器用の速度トピックはこのlaunchでは切り替えません。
Castella側の起動は引き続きNSKの `/odom` を速度入力に使い、NSKのTFは無効にします。

## タイムスタンプと地図品質

現在のHesai設定`use_timestamp_type: 1`は、点群とIMUを同じホスト受信時刻系で配信するため、そのまま動作します。ただし点ごとの時刻はUDPパケット単位であり、厳密なレーザー発光時刻ではありません。地図品質を上げる場合は、LiDARとPCのPTP同期を確立したうえで`use_timestamp_type: 0`へ移行することを推奨します。

現在のCastella側Hesai設定は`distance_correction_flag: false`で、点群の基準は光学中心、`r_bl/t_bl`のURDF取付基準はLiDAR幾何原点です。このままでもLIOと同じ設定で作った地図への再ローカライズはできますが、車体基準にcm級のずれが残り得ます。本番地図を作る前に`distance_correction_flag: true`へ切り替え、点群形状と取付TFを実機確認することを推奨します。`transform_flag`は`false`のままとし、取付変換は`r_bl/t_bl`だけで行います。

JT-128のfiretime correction fileをHesaiドライバへ設定すると、channel発光時刻に基づくazimuth補正も利用できます。ただしPointCloud2の`timestamp`は引き続きパケット単位です。まず低速で動作を確認し、その後の地図品質評価項目として扱ってください。

最初の姿勢を配信する前にIMU/LiDARの前進gapが設定上限（既定100 ms / 1秒）を超えた場合は、ノードを終了せず初期化をやり直します。両センサの待機queue、IMU初期化sample・bias、途中の地図、予測履歴を破棄し、欠落後の点群と、その走査全体を覆う連続したIMUがそろうまで待ちます。IMU初期化に必要なsample数も最初から数え直します。待機中は姿勢・TFを配信せず、`/fastlio2/processing/diagnostics` に `startup_waiting_after_imu_gap` / `startup_waiting_after_lidar_gap`、欠落秒数、再試行回数、入力再開の基準時刻を記録します。これは起動直後の一時的な欠落への対応で、センサ欠落の原因そのものを修正するものではありません。

最初の姿勢を配信した後の上限超過は、従来どおりLIOノードを終了します。走行中の座標系を自動で初期化し直すことはしません。時刻が0.5秒を超えて巻き戻った場合も、起動段階に関係なく終了します。先頭へ巻き戻してrosbagを再生するときや推定開始後にセンサ通信が長時間途切れたときはノードも再起動してください。ログのtimestamp gapは隣接する受理sampleの時刻差であり、時計自体の不具合を確定するものではありません。重複または軽微な順序逆転のIMUは破棄し、センサ待ちqueueには上限を設けています。初期加速度がSI単位として不自然な場合も、重力方向を壊す前に初期化を中止します。

`startup_continuous_input_seconds`（YAML / ROSパラメータ）は最初の姿勢公開までに必要な連続入力時間です。省略時0秒、Castellaの `jt128.yaml` は5秒です。内部の地図初期化が完了しても、この時間に達するまでは姿勢・点群・TFを公開せず、欠落時の初期化やり直しを許容します。欠落があれば連続時間も数え直します。`startup_waiting_for_continuous_input` 診断で経過時間と必要時間を確認できます。IMUのDDS受信履歴は2000件に拡張していますが、欠落上限100 msは変更していません。

## テスト対象

- JT-128の非アライン`FLOAT64 timestamp`を含むPointCloud2変換
- Ousterの相対nanosecond形式との互換性
- 点時刻の整列、scan終端、欠損・破損・非有限入力の拒否
- LiDAR Jacobian、SO(3)演算、IMU単位、外部変換、出力frame
- 起動中の反復欠落・初期化途中の欠落からの再開、古いscan/biasの破棄、推定開始後の欠落と時刻逆行による終了
- 合成地図に対するICP自己位置推定と不正地図・不良初期値の拒否

FAST-LIO2は非常停止や衝突回避を提供しません。実機で地図生成を行う際は、周囲、非常停止手段、別系統の衝突防止機能を確認し、最初は低速で操作してください。
