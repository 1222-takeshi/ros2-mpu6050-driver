# 実機構成と検証条件の記録

管理Issue: [#61](https://github.com/1222-takeshi/ros2-mpu6050-driver/issues/61)

この文書は構成・測定方法・合否基準を記録するためのひな形であり、実機の合格報告ではない。
未確定の値は「未確認」のまま残し、数値基準を決めてから実測する。

## 確認済みの目的

- 既存台車を使ったROS 2上での倒立制御を目標とする。現在の車体が倒立用かは別途確認する。
- 最初の実機到達点は静止状態での倒立維持とする。
- IMUデータを車輪オドメトリ等と融合し、自己位置推定のソースにも使用する。
- 実機が存在することと、時間性能・倒立・推定精度の達成は区別する。

## 実機構成

| 項目 | 実機での値・根拠 |
| --- | --- |
| Raspberry Pi型番・CPU/電源設定 | 未確認 |
| OS・kernel・ROS 2・RMWの版 | 実機は未確認。本ドライバの基準環境はUbuntu 22.04 / Humble、台車のDocker定義はJazzy |
| 台車側ソフトのリポジトリ・commit | `1222-takeshi/diff-drive-robot` / `83e0a298de465038372c1db49f7cabfbb33854fa`。実機に配備した版は未確認 |
| モータ・モータドライバ・エンコーダ | BOMは型番不明のDCギヤモータ2台とL9110S。エンコーダの記録とコードに差異あり（下記） |
| MPU6050の取付姿勢・軸・取付位置 | 未確認 |
| I2Cバス・速度・配線・アドレス | MPU6050側は未確認。本ドライバは0x68。台車の既存IMUは別型番・Pico接続（下記） |
| センサ内部の測定周期・フィルタ・測定範囲 | 未確認。換算係数の前提と実設定が一致することを確認する |
| 姿勢推定・制御・自己位置推定の担当パッケージ | 台車に`ddr_base_driver`とPico firmwareあり。倒立用の担当・接続は未確定 |
| 同時に動かすノードと通常負荷 | 未確認 |

実機の値には、確認方法、確認日時、設定ファイルまたは取得ログの保存先を併記する。

## 台車リポジトリから確認した接続と未解決点

2026-10-21にユーザー指定の`diff-drive-robot/`を確認した。
以下は上記commitの追跡済みファイルに基づく記録であり、現物・書込み済みfirmware・実測値の確認ではない。

| 項目 | 記録された構成・実装と根拠 |
| --- | --- |
| 既存IMU | [BOM](https://github.com/1222-takeshi/diff-drive-robot/blob/83e0a298de465038372c1db49f7cabfbb33854fa/hardware/bom.yaml)はMiniIMU-9 v5を所持・接続済みと記録。MPU6050ではなくLSM6DS33とLIS3MDL |
| 配線・I2C設定 | [pinmap](https://github.com/1222-takeshi/diff-drive-robot/blob/83e0a298de465038372c1db49f7cabfbb33854fa/hardware/pinmap.yaml)はPico I2C0 GP8/GP9、アドレス0x6B/0x1E。[firmware設定](https://github.com/1222-takeshi/diff-drive-robot/blob/83e0a298de465038372c1db49f7cabfbb33854fa/firmware/pico/src/config.h)は400kHz |
| IMU読取内容 | [imu.cpp](https://github.com/1222-takeshi/diff-drive-robot/blob/83e0a298de465038372c1db49f7cabfbb33854fa/firmware/pico/src/imu.cpp)はgyroとmagを読みyawを計算。加速度の読取はなく、pitch/rollは常に0を代入 |
| Picoからの報告 | [main.cpp](https://github.com/1222-takeshi/diff-drive-robot/blob/83e0a298de465038372c1db49f7cabfbb33854fa/firmware/pico/src/main.cpp)は100ms間隔を設定。`IMU yaw pitch roll gx gy gz`に取得timestamp・sequence・加速度は含まれない。10Hzは設定からの換算で、実測周期ではない |
| ROS側の取込 | [base_driver_node.py](https://github.com/1222-takeshi/diff-drive-robot/blob/83e0a298de465038372c1db49f7cabfbb33854fa/ros2_ws/src/ddr_base_driver/ddr_base_driver/base_driver_node.py)はUSB serialまたはTCPで受信し、yawとgzだけを使用。`imu`へyawのみの姿勢とz角速度を配信し、timestampはホストでのpublish時に付与 |
| 既存odometry | 同nodeは指令速度を積分し、新鮮なIMU yawがあれば方位に使用（受信時刻から1秒未満）。`ENC`報告をparseしていないため、車輪実測odometryとは区別する |
| モータ指令・制御 | 同nodeの既定指令送信は5Hz、read timerは20ms。Picoの車輪速度PID設定は100ms、指令timeoutは300ms。倒立制御の周期・欠落許容値としては未承認 |
| 車体 | [URDF](https://github.com/1222-takeshi/diff-drive-robot/blob/83e0a298de465038372c1db49f7cabfbb33854fa/ros2_ws/src/ddr_description/urdf/ddr.urdf.xacro)とBOMは駆動輪2個とキャスターを定義。倒立用への改造か別車体か、重心・慣性・取付軸は未確認 |
| ROS 2環境 | [Dockerfile](https://github.com/1222-takeshi/diff-drive-robot/blob/83e0a298de465038372c1db49f7cabfbb33854fa/docker/Dockerfile)は`ros:jazzy-ros-base`。本ドライバのHumble CI通過はJazzy統合の検証を代替しない |

既存経路は`MiniIMU → Pico → serial/TCP → ddr_base_driver`であり、
本ドライバの`MPU6050 → ホストI2C → /imu/data_raw`を購読する接続は確認できなかった。
既存nodeがpublishするtopicをremapするだけでは外部IMUの入力経路にならない。
利用するセンサ・取得ホスト・推定/制御担当を確定してから、接続とtimestamp・単位・TF・欠落時動作を実装する。

設定間の不整合も実機照合が必要であり、現物の値や動作許可として採用しない。

- [robot.yaml](https://github.com/1222-takeshi/diff-drive-robot/blob/83e0a298de465038372c1db49f7cabfbb33854fa/config/robot.yaml)はencoder無効・slit数/車輪寸法未記入、pinmapはLM393到着待ち、BOMは読取センサ未所持。一方firmwareは24slits・車輪径65mmを定義し、encoder/PIDを初期化する。
- [QWEN_CONTEXT.md](https://github.com/1222-takeshi/diff-drive-robot/blob/83e0a298de465038372c1db49f7cabfbb33854fa/QWEN_CONTEXT.md)はPWM上限25%、robot.yamlとfirmwareは約75%。配備版と承認された上限は未確認であり、この文書では変更しない。
- 台車の[走行評価シナリオ](https://github.com/1222-takeshi/diff-drive-robot/blob/83e0a298de465038372c1db49f7cabfbb33854fa/docs/driving-scenarios.md)にはS01〜S04の暫定基準がある。#62の比較条件を決める際の候補とし、静止倒立の合否基準に転用しない。

## ドライバ側の接続契約

以下はソフトウェアの既定値であり、実機での実測値ではない。
実行時のパラメータ・remap・TF・QoSを確認し、記録したcommitと対応させる。

| 項目 | 現在の契約 |
| --- | --- |
| 生IMU | `output` / `sensor_msgs/msg/Imu`。付属launchの既定remapは`/imu/data_raw` |
| 角速度・加速度 | bias補正後の`rad/s`・`m/s^2` |
| 姿勢 | 生IMUには推定姿勢を含めず、`orientation_covariance[0] = -1` |
| covariance | 既定値の全ゼロは未知を表す。統合時は実測値・根拠を記録する |
| `roll_pitch` | 加速度由来の簡易角度で、単位はdegree。動作中の倒立制御への適合性は未検証 |
| frame | `frame_id`の既定値は`imu`。取付TFと制御・推定側の軸を一致させる |
| timestamp | ソフトウェアのI2C読み取り完了後に得るROS timeをIMUとroll_pitchで共有する。センサ内部の測定時刻ではない |
| 配信周期 | `publish_rate_hz`の既定値は100.0。タイマーはms単位であり、実測周期やセンサ内部の測定周期とは区別する |
| QoS | 両publisherともReliable / KeepLast / depth 10。実行時も確認する |

異常データの扱いは[#54](https://github.com/1222-takeshi/ros2-mpu6050-driver/issues/54)で管理する。
制御側には別途、データ停止・古いデータを検知するtimeoutと停止・復帰の契約が必要である。

## 測定点とclock

1. I2C読み取りの開始・完了と、メッセージのheader timestampを記録する。
2. 姿勢推定での受信・計算完了を記録する。
3. 制御計算で使用した入力timestamp、計算開始・完了を記録する。
4. モータ指令の発行を記録する。実際のモータ応答を観測できない場合は、その遅延を未観測として残す。

これらの測定点を取得する実装・設定は、担当パッケージを確認してから用意する。
現在のdiagnosticsだけで全経路を測定済みとは扱わない。

headerからのデータ年齢は、同じclockに基づく受信・制御時のROS timeとの差で計算する。
同一ホスト内の処理時間には単調増加clockを使い、異なるclockの値を直接減算しない。
複数ホストの場合はclock同期とその誤差も確認する。負の時間差やclock jumpは異常として記録する。
headerからの年齢には、センサ内部の測定からI2C読み取り完了までの遅延を含むとは限らない。

## 実測前に決める合否基準

| 項目 | 値・定義・根拠 |
| --- | --- |
| 制御周期とIMU取得・配信周期 | 未確認 |
| 各区間と全経路の最大遅延、許容jitter | 未確認 |
| 制御時に許容する最大データ年齢 | 未確認 |
| 許容欠落数・連続欠落時間、停止・復帰条件 | 未確認 |
| 静止倒立の維持時間、姿勢角と参照値、誤差の指標 | 未確認 |
| 試行回数・成功判定・中断条件 | 未確認 |
| 自己位置推定の走行条件・独立参照値・位置/方位誤差基準 | 未確認 |

100Hzは測定開始時の候補とし、倒立制御への十分性を既定値から推定しない。
基準と評価方法を試験前に固定し、未確定値がある間は実機合格を宣言しない。

## 記録と次の検証

試行ごとにID、日時、実機構成、各パッケージのcommit、パラメータ、負荷、測定時間、
基準の版、結果、raw dataの保存先とchecksumを残す。大きなrosbagは外部保存する。
変更前の記録を保持し、IMU有無・性能変更前後は同じ条件で比較する。
未達・未評価・clock異常・観測できない遅延をそのまま記録し、合格に読み替えない。

IMU単体の周期・鮮度は[#41](https://github.com/1222-takeshi/ros2-mpu6050-driver/issues/41)、
自己位置推定の実機評価は[#62](https://github.com/1222-takeshi/ros2-mpu6050-driver/issues/62)、
全経路と静止倒立の実機評価は[#63](https://github.com/1222-takeshi/ros2-mpu6050-driver/issues/63)で追跡する。
