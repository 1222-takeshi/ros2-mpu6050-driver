# 実機構成と検証条件の記録

管理Issue: [#61](https://github.com/1222-takeshi/ros2-mpu6050-driver/issues/61)

この文書は構成・測定方法・合否基準を記録するためのひな形であり、実機の合格報告ではない。
未確定の値は「未確認」のまま残し、数値基準を決めてから実測する。

## 確認済みの目的

- 倒立用の別車体を使い、ROS 2上で倒立制御を行う（ユーザー確認済み）。
- 最初の実機到達点は静止状態での倒立維持とする。
- IMUデータを車輪オドメトリ等と融合し、自己位置推定のソースにも使用する。
- 実機が存在することと、時間性能・倒立・推定精度の達成は区別する。

## 実機構成

| 項目 | 実機での値・根拠 |
| --- | --- |
| Raspberry Pi型番・CPU/電源設定 | 未確認 |
| OS・kernel・ROS 2・RMWの版 | 実機は未確認。倒立側のREADMEと本ドライバの基準環境はROS 2 Humble。Pi型番・OSの記録には差異あり（下記） |
| 台車側ソフトのリポジトリ・commit | `1222-takeshi/inverted_pendulum` / `ef2951064bdc1a0c5f59d699bb90fbb0541617a3`。実機に配備した版は未確認 |
| モータ・モータドライバ・エンコーダ | BOMはDYNAMIXEL XL330-M077-T ×2、U2D2接続。wheel stateはサーボの現在位置・速度を読む設計（下記） |
| MPU6050の取付姿勢・軸・取付位置 | 未確認 |
| I2Cバス・速度・配線・アドレス | 倒立側の配線図はPi GPIO2/3のI2C、MPU6050 0x68。実機と内部設定は未確認 |
| センサ内部の測定周期・フィルタ・測定範囲 | 未確認。換算係数の前提と実設定が一致することを確認する |
| 姿勢推定・制御・自己位置推定の担当パッケージ | `balance_bot_hardware`、`balance_bot_control`、`dynamixel_driver`。設定と未接続箇所は下記 |
| 同時に動かすノードと通常負荷 | 未確認 |

実機の値には、確認方法、確認日時、設定ファイルまたは取得ログの保存先を併記する。

## 倒立台車リポジトリの接続と未解決点

2026-10-21にユーザーが対象を`diff-drive-robot/`から`inverted_pendulum/`へ訂正した。
倒立用は別車体であり、diff-drive側のPico・L9110S・キャスター・10Hz報告は倒立側の構成ではない。
以下は上記commitの追跡済みソースの記録であり、現物・配備版・今回の実測確認ではない。

| 項目 | 倒立側の実装・設定と根拠 |
| --- | --- |
| IMU入力経路 | [real_input.launch.py](https://github.com/1222-takeshi/inverted_pendulum/blob/ef2951064bdc1a0c5f59d699bb90fbb0541617a3/src/balance_bot_hardware/launch/real_input.launch.py)は本ドライバの`output`を`/imu/data_raw`へremapし、`imu_unit_adapter_node`へ渡す |
| 正規化と時刻 | [adapter](https://github.com/1222-takeshi/inverted_pendulum/blob/ef2951064bdc1a0c5f59d699bb90fbb0541617a3/src/balance_bot_hardware/src/imu_unit_adapter_node.cpp)はheader stampを保持しframeを`imu_link`へ変更、姿勢は未提供とする。frame名変更は取付軸の回転変換ではない |
| 単位 | 実機用[hardware_input.yaml](https://github.com/1222-takeshi/inverted_pendulum/blob/ef2951064bdc1a0c5f59d699bb90fbb0541617a3/src/balance_bot_bringup/config/real/hardware_input.yaml)は`gyro_in_degrees=false`・`accel_in_g=false`。本ドライバのSI出力を二重変換しない |
| 姿勢推定・制御 | [balance_controller_node.cpp](https://github.com/1222-takeshi/inverted_pendulum/blob/ef2951064bdc1a0c5f59d699bb90fbb0541617a3/src/balance_bot_control/src/balance_controller_node.cpp)が`/imu`と`/joint_states`を使用。実機[controller設定](https://github.com/1222-takeshi/inverted_pendulum/blob/ef2951064bdc1a0c5f59d699bb90fbb0541617a3/src/balance_bot_bringup/config/real/balance_controller.yaml)は`use_imu_orientation=false`でcomplementary filterを使う |
| 周期 | controller timerは10ms、filterと制御のdtは固定0.01秒。IMU/制御の実測周期と同一とは限らず、実測dt対応は[#219](https://github.com/1222-takeshi/inverted_pendulum/issues/219) |
| 車輪入力 | [joint state node](https://github.com/1222-takeshi/inverted_pendulum/blob/ef2951064bdc1a0c5f59d699bb90fbb0541617a3/src/balance_bot_hardware/src/dynamixel_joint_state_node.cpp)はXL330から位置・速度を読む。実機設定はID 1/2、100Hz、zero_on_startup=true。ただしreal_input launchにnodeの起動が欠けている（[#217](https://github.com/1222-takeshi/inverted_pendulum/issues/217)） |
| モータ出力 | [output設定](https://github.com/1222-takeshi/inverted_pendulum/blob/ef2951064bdc1a0c5f59d699bb90fbb0541617a3/src/balance_bot_bringup/config/real/dynamixel_output.yaml)は10ms write、command timeout 200ms、`torque_on_startup=false`。これらはセンサ停止を検知する保証ではない |
| 依存ドライバ | [depends.repos](https://github.com/1222-takeshi/inverted_pendulum/blob/ef2951064bdc1a0c5f59d699bb90fbb0541617a3/depends.repos)は本ドライバの`main`をimportする。再現時は取得した実SHAと配備版も記録する |

倒立側は既存MPU6050を使用することをユーザー確認済み。
IMU取付姿勢・内部設定・配備版は引き続き実機と照合する。

実機照合と後続実装が必要な点:

- [BOM](https://github.com/1222-takeshi/inverted_pendulum/blob/ef2951064bdc1a0c5f59d699bb90fbb0541617a3/BOM.md)と[配線図](https://github.com/1222-takeshi/inverted_pendulum/blob/ef2951064bdc1a0c5f59d699bb90fbb0541617a3/docs/wiring/rpi_wiring_v1.md)はPi 4、[CLAUDE.md](https://github.com/1222-takeshi/inverted_pendulum/blob/ef2951064bdc1a0c5f59d699bb90fbb0541617a3/CLAUDE.md)にはPi 3B+ / Ubuntu 22.04の記載もある。実機の型番・OSを確認する。
- hardware packageの既定configはbaud 1000000、bringup側の実機input/output設定は57600。配備版との整合は[#211](https://github.com/1222-takeshi/inverted_pendulum/issues/211)で管理する。
- controllerは一度受信したことをflagで記録するが、入力停止時の鮮度timeoutは未実装。driverの異常値破棄だけでは古い入力による制御を防げない。watchdog/停止は[#178](https://github.com/1222-takeshi/inverted_pendulum/issues/178)で管理する。
- [PLAN.md](https://github.com/1222-takeshi/inverted_pendulum/blob/ef2951064bdc1a0c5f59d699bb90fbb0541617a3/PLAN.md)のM2目標は静止倒立30秒。今回の達成値ではなく、姿勢誤差・試行数・時間性能の合否基準は追加定義が必要。

## ドライバ側の接続契約

### 2026-10-24のソフトウェア準備（現物・配備版は未確認）

上の2026-10-21の調査と表は固定SHAの履歴であり、現在の未実装項目一覧ではない。
以降の変更は次のとおり。実機への配備や性能達成を示すものではない。

| 項目 | ソフトウェア上の到達点 |
| --- | --- |
| 起動経路 | 倒立側[PR #236](https://github.com/1222-takeshi/inverted_pendulum/pull/236)でjoint state bridgeをreal_inputから起動。現物のID/baud/zero_on_startupは未確認 |
| IMU・制御dt | 倒立側[PR #252](https://github.com/1222-takeshi/inverted_pendulum/pull/252)/[#256](https://github.com/1222-takeshi/inverted_pendulum/pull/256)/[#257](https://github.com/1222-takeshi/inverted_pendulum/pull/257)でclock選択、source間隔のCF積分、制御積分・出力平滑化の実測dtに対応。IMU alphaの時間正規化や倒立合格は別途 |
| 入力停止 | 倒立側[PR #258](https://github.com/1222-takeshi/inverted_pendulum/pull/258)でopt-inの入力年齢・進行監視とWAIT/zero latchを実装。input_timeout_sec=0が既定で無効。実機budgetは未確定。復帰はprocess restartとfresh入力/startup条件を必要とする |
| 設定の適用 | 本repo [PR #76](https://github.com/1222-takeshi/ros2-mpu6050-driver/pull/76)でrate/frame/bias/covarianceの6設定をstartup-only化。runtime変更は拒否し、変更には再起動が必要。起動時fallbackは従来どおり |
| 記録・集計 | 倒立側[PR #261](https://github.com/1222-takeshi/inverted_pendulum/pull/261)/[#262](https://github.com/1222-takeshi/inverted_pendulum/pull/262)に診断付きcapture profileとJSON/Markdown timing report。header周期と診断snapshotを区別し、budgetを自動推奨しない |
| 旧診断例 | 本repo [PR #77](https://github.com/1222-takeshi/ros2-mpu6050-driver/pull/77)で非installのmpu6050_sample.pyを撤去。別I2C readerのdeg/s/gとROS SI出力を取り違えないよう、ROS経路へ一本化 |

測定前に配備SHAとsource SHAを照合し、node ROS-clock domainとheaderの意味を確認する。
複数ホストでは同期誤差も記録する。診断値・bag receiptからセンサ内部/モータ応答の
未観測latencyを作らず、観測最大値だけを安全な許容年齢として採用しない。
入力監視は実行可能control callbackでzeroを送る機構であり、paused ROS timer、
blocked executor、crashや物理的停止への保証ではない。torque-offでの確認、
Bus Watchdog/output timeout/physical emergency stopの確認を別に行う。

Pi型番/OS、取付姿勢・軸、I2C内部設定、ID/baud、通常負荷、clock同期、
許容年齢・jitter/欠落、姿勢誤差・試行数の合否基準は引き続き実機での照合が必要。
下の未確認欄を埋めるまでは実機合格を宣言せず、real watchdog閾値を有効化しない。

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
制御側にはopt-inのtimeout/停止ラッチが実装済みである。実機budgetとclock domain、
停止・再起動復帰の運用を確認し、torque-off条件で有効化・検証する必要がある。

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
