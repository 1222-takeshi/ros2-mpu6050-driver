# 開発ロードマップ（提案）

更新日: 2026-10-21 / 管理Issue: [#57](https://github.com/1222-takeshi/ros2-mpu6050-driver/issues/57)

## 目指す状態

Raspberry Pi上でMPU6050を安定運用し、異常とデータの鮮度を観測でき、
下流の状態推定へ接続できるROS 2ドライバを目指す。
当面の検証対象は既存のUbuntu 22.04 / ROS 2 Humble構成とする。
100Hzは目標値であり、実機で達成済みという意味ではない。

目的の優先順位は「データの信頼性 → 実機での測定 → 測定に基づく改善 → 統合」とする。
以下は実施順序の提案であり、各段階の完了は対応Issueの検証結果で判断する。

## 実施順序と完了条件

| 段階 | 対象 | 完了条件・次へ進む条件 |
| --- | --- | --- |
| 1. 既存PRの整理 | [#50](https://github.com/1222-takeshi/ros2-mpu6050-driver/issues/50) / [PR #51](https://github.com/1222-takeshi/ros2-mpu6050-driver/pull/51): I2C初期化失敗、[#44](https://github.com/1222-takeshi/ros2-mpu6050-driver/issues/44) / [PR #45](https://github.com/1222-takeshi/ros2-mpu6050-driver/pull/45): frame_id、[#47](https://github.com/1222-takeshi/ros2-mpu6050-driver/issues/47) / [PR #48](https://github.com/1222-takeshi/ros2-mpu6050-driver/pull/48): launch設定、[#58](https://github.com/1222-takeshi/ros2-mpu6050-driver/issues/58) / [PR #52](https://github.com/1222-takeshi/ros2-mpu6050-driver/pull/52): timestamp | 2名以上のレビューとHumbleのbuild/testを通過する。静的解析は実行して変更前と比較し、新規違反を追加しない。launch設定は実起動後の型・値まで確認し、統合後にも全体を検証する。 |
| 1a. 静的解析の整備 | [#59](https://github.com/1222-takeshi/ros2-mpu6050-driver/issues/59) | 変更前から存在するlint違反を解消し、CIで静的解析の再発を検出する。機能変更と分離して進める。 |
| 2. 異常データの扱い | [#54](https://github.com/1222-takeshi/ros2-mpu6050-driver/issues/54) | 無効サンプルのpublish抑止、診断、拒否数、正常復帰、鮮度・周期計測の契約を決め、正常・境界・異常・復帰をmockで検証する。実機の故障時確認は別途記録する。 |
| 3. 変更前の実機測定 | [#41](https://github.com/1222-takeshi/ros2-mpu6050-driver/issues/41) | 通常ロボット負荷で100Hz目標の周期・jitter・鮮度とdiagnosticsを記録する。Pi/OS/ROS、I2C設定、負荷、実行コマンド、commit SHA、測定時間を残す。 |
| 4. 測定に基づく改善 | [#46](https://github.com/1222-takeshi/ros2-mpu6050-driver/issues/46): バースト読み取り、[#53](https://github.com/1222-takeshi/ros2-mpu6050-driver/issues/53): QoS比較 | 段階3の記録を保存してから、変更を一つずつ比較する。6軸変換・不完全読み取り・診断の回帰がなく、同じ条件で効果と互換性を説明できる。 |
| 5. 状態推定への接続 | READMEのEstimator Setupを利用。具体的な利用先が決まったら独立Issueを作成 | 実測bias/covariance、TF、重力の扱い、topic/QoS、timestampを確認し、静止・回転・通常走行の記録で下流の動作を検証する。 |

段階1の統合順序は #51 → #45 → #48 → #52 を候補とする。
#45と#48は同じlaunchファイルを変更するため、統合時に全引数が残ることを確認する。
重複する[#49](https://github.com/1222-takeshi/ros2-mpu6050-driver/issues/49)は#50へ集約し、
I2C初期化失敗の修正を二重実装しない。

段階1では、mainの固定snapshotでも再現した既存lint失敗45件を#59で追跡する。
静的解析全体の成功とは報告せず、同じ条件で失敗箇所・内容が増えていないことを確認する。
既存違反の解消とCIでの継続検証は段階1aの完了条件とする。

## 性能・互換性の判断

- #46と#53は独立した変更として扱い、変更前後を同じ負荷・測定時間で比較する。
- バースト読み取りはshort readや軸の順序をmockで確認し、実配線でも検証する。
- QoSは現行Reliableとの互換性を評価する。比較前にBest Effortへ既定値を変更しない。
- #41の測定前に合否基準を記録する。測定結果が目標を満たさなければ未達として残す。
- source/raw dataと変更前の記録を保持する。大きなrosbag等は外部保存し、小さなreport/manifestに保存先・checksum・条件を残す。
- CI/mockの成功は実機100Hz達成や故障時の安全性を保証しない。実機確認が終わるまで実機項目を完了扱いにしない。

## 別途判断する項目

- [#55](https://github.com/1222-takeshi/ros2-mpu6050-driver/issues/55): 旧Pythonサンプルは、現行ROSドライバの検証に必要かを確認して廃止または診断ツール化を決める。主経路の検証と並行して判断できる。
- 他のROS 2ディストリビューション対応、backendの置き換え、配布方法の拡張は、利用先と保守範囲が決まってから独立Issueにする。
- 再接続・自動復帰の機能追加は、現行の失敗時契約と実機での要求を確認してから設計する。

## Issue・PRの運用

一つの未完了Issueに対して一つの小さな変更を進める。
PRには目的、tests、静的解析、validation、実機未確認項目を記載する。
レビューとQuality Gateを通過してから統合し、Issueは完了条件を満たした証跡に基づいて閉じる。
この文書の統合は、各機能Issueや実機測定の完了を意味しない。
