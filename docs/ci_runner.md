# DGX Spark CI

`spark-d725` 上のrepository専用runner `mpu6050-driver-dgx-spark` で実行する。
labelsは `self-hosted / Linux / ARM64 / dgx-spark`。倒立repositoryのrunnerと
ディレクトリ・user serviceを分け、同一hostでもROS通信はcontainerのloopbackへ制限する。

public repositoryなので、GitHub側のfork workflow承認設定を
`all_external_contributors` にする。YAMLの条件はPR自身が変更できるため、
サーバー側の設定も必須である。fork workflowを承認してこのrunnerへ流さない。
YAMLでもfork PRをskipし、`pull_request_target` は使用しない。
外部変更はreview後にmaintainer管理branchへ取り込み、通常のPRとして検証する。
repositoryへのwrite権限はこのhostでコードを実行できる信頼境界として扱う。
移行時にwrite/admin権限を持つのはowner `1222-takeshi` のみであることを確認した。
collaboratorや承認設定を変更するときは、この信頼境界を再確認する。

workflowはdigest固定のJammy containerにHumbleを導入し、既存の全登録testとlintを実行する。
workspaceはsource checkoutの外の一意なRUNNER_TEMP配下へ置く。
失敗時もtest resultsをuploadし、upload後にこのjobのworkspaceを削除する。
mock testとARM64 buildの成功は、Pi実配線のI2C性能や倒立達成の検証ではない。

## Runner運用

- SSH: `ssh tier4-x1@spark-d725`
- install directory: `/home/tier4-x1/actions-runner-mpu6050-driver`
- user unit: `actions-runner-mpu6050-driver.service`
- 確認: `systemctl --user is-active actions-runner-mpu6050-driver.service`
- 自動起動: user serviceのenableと既存のLinger設定を使用する。再起動試験とは区別する。

runner登録はrepositoryの一時tokenで行い、tokenやrunner認証ファイルを
repository、log、artifactへ保存しない。runner directoryは所有者のみアクセス可能にする。
runner binaryは公式releaseのSHA256と照合する。

変更時はPRとmerge後のmainで、runner名・ARM64環境、全test/lint、artifact upload、
一時workspace削除を確認する。失敗時は診断を保持してから独立した変更として修正する。
