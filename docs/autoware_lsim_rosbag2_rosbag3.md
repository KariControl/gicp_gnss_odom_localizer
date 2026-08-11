# Autoware lSIMでROSBAG2・ROSBAG3を動かす手順

## 1. 対象と試験範囲

この手順は、次の2つのbagでGICP–GNSS Odom LocalizerをAutoware 1.9.0の
Logging Simulation（lSIM）へ接続するためのものです。

| 呼称 | パス | 時間 | PointCloud2 | IMU | NMEA |
|---|---|---:|---:|---:|---:|
| ROSBAG2 | `rosbag/output_pointcloud2` | 210.757 s | 4,216 | 42,153 | 8,430 |
| ROSBAG3 | `rosbag/output_pointcloud3` | 213.635 s | 4,273 | 42,727 | 8,546 |

これは**localization-interfaceだけのlSIM**です。Autoware標準のmap、localization、
perception、planning、control、system、API、sensor driverは無効にし、この
localizerの出力をAutoware標準localization topicへ変換します。地図を使った経路計画、
閉ループ走行、車両制御の合格を示す試験ではありません。

Autoware 1.9.0は`autoware_launch` 0.52.0を使用します。このリポジトリのlaunchが
渡すmodule選択引数は0.52.0の`autoware.launch.xml`と一致しています。

- [Autoware 1.9.0 release](https://github.com/autowarefoundation/autoware/releases/tag/1.9.0)
- [Autoware 1.9.0の固定リポジトリ一覧](https://raw.githubusercontent.com/autowarefoundation/autoware/1.9.0/repositories/autoware.repos)
- [autoware.launch.xml 0.52.0](https://raw.githubusercontent.com/autowarefoundation/autoware_launch/0.52.0/autoware_launch/launch/autoware.launch.xml)
- [公式Logging Simulation launch](https://raw.githubusercontent.com/autowarefoundation/autoware_launch/0.52.0/autoware_launch/launch/logging_simulator.launch.xml)

## 2. `hesai-rosbag23`プロファイル

ROSBAG2/3には`/tf`、`/tf_static`、`/clock`がありません。汎用profileのままでは
strict deskewとGNSSアンテナ補正に必要なTFが得られないため、次の条件をひとつの
profileに固定しています。

### 入力topic

```text
/pandar_points_ex            -> /points_raw
/sensor/imu/data_raw         -> /imu
/sensor/gnss/nmea_sentence   -> /nmea_sentence
```

raw packetの`/sensor/lidar_front/pandar_packets`は再生しません。処理済み
`PointCloud2`が存在し、raw packetはlocalization入力に不要だからです。

### Static TF

値は`script/run_vehicle_localizer_hesai_nmea_gnss_no_snow_lp.sh`と同じです。

```text
base_link -> lidar/0 : xyz=(0, 0,  0.0000), rpy=(0,       0, 0)
base_link -> imu     : xyz=(0, 0, -0.1874), rpy=(3.14159, 0, 0)
base_link -> gnss/0  : xyz=(0, 0, -0.1326), rpy=(0,       0, 0)
```

これらは提供された同一rigの較正値です。bagには較正真値が含まれないため、bagだけで
数値を再較正することはできません。Autowareの`sample_sensor_kit` TFとはframe名も
設置条件も異なるので、`--launch-vehicle`を同時に指定してはいけません。

### Estimator設定

ROS 2単体の全区間試験と同じ設定を使います。

```text
pure_imu_undistortion/param/param_xt.yaml
pure_lidar_gyro_odometer/param/param_xt_lidar_imu_only.yaml
pure_nmea_gnss_conversion/param/param.yaml
pure_gnss_map_odom_fusion/param/param.yaml
```

PointCloud2にはFLOAT64の絶対時刻field `time_stamp`があり、1 scanは約49.3 msです。
内部IMU deskewを有効にし、linear time fallbackと並進deskewは無効のままにします。
したがって、このprofileでは`--already-deskewed`を使用できません。

### GNSSローカル原点

汎用NMEA設定の東京原点は収録地点から約274 km離れています。profileでは両bagの
有効GGA包絡中心を共通原点に上書きします。

```yaml
map_origin.latitude: 35.1254574925
map_origin.longitude: 136.8226007017
```

全有効GGAは原点から概算77 m以内に収まります。mapを起動しない比較用のローカル
原点です。将来PCD/Lanelet2 mapと統合するときは、そのmapのprojector設定に合わせて
置き換えてください。

### Simulation clock

`ros2 bag play --clock 100.0`相当で100 Hzの`/clock`を生成します。全localizer、
Adapter、Autoware側nodeは`use_sim_time=true`です。rosbag2既定の低いclock rateでは
fusion timerが同じstampを複数回出すことがあるため、Adapterは同一stampを正常な
duplicateとして抑制し、真の時刻逆行だけをrejectします。

## 3. 必要環境

ホストに必要なのは次のものです。

- Docker Engine
- Docker Compose v2 plugin（`docker compose`コマンド）
- ROSBAG2/3を読み出せるディスク容量
- official Autoware imageと依存packageを初回に取得できるネットワーク

ホスト側のROS 2、Autoware source workspace、CUDA GPU、PCD map、Lanelet2 mapは
不要です。headless・CPU-onlyが既定です。

今回の実機では最終overlay imageのvirtual sizeは約2.77 GBでしたが、公式baseとbuild
cacheを含むDocker使用量は約24.3 GBでした。初回build前に最低30 GB程度の空きを推奨
します。出力MCAPは1 runあたり約136--151 MiBです。

リポジトリrootで事前確認します。

```bash
cd /path/to/gicp_gnss_odom_localizer

docker --version
docker compose version
docker info

test -f rosbag/output_pointcloud2/metadata.yaml
test -f rosbag/output_pointcloud3/metadata.yaml
```

`docker info`が`/var/run/docker.sock: permission denied`になる場合は、Docker groupを
追加した後のログインセッションへ入り直すか、`newgrp docker`を実行します。現在の
shellだけで実行する場合は、4章のコマンド全体を次のように`sg docker -c`で囲めます。

```bash
sg docker -c "ROS_DOMAIN_ID=82 ./script/run_autoware_lsim_docker.sh \
  --bag '$PWD/rosbag/output_pointcloud2' \
  --profile hesai-rosbag23 \
  --tracking-mode scan_to_scan \
  --run-name rosbag2_lsim"
```

他のROS 2 graphとの混線を避けるため、未使用の`ROS_DOMAIN_ID`をrunごとに指定して
ください。

## 4. 実行手順

### 4.1 設定だけを確認する

Dockerを起動せず、解決後のprofileとCompose commandを表示できます。

```bash
ROS_DOMAIN_ID=82 \
  ./script/run_autoware_lsim_docker.sh \
  --bag "$PWD/rosbag/output_pointcloud2" \
  --profile hesai-rosbag23 \
  --run-name rosbag2_lsim \
  --dry-run
```

表示に次が含まれることを確認します。

```text
Dataset profile:     hesai_rosbag23
Clock frequency:     100.0 Hz
GNSS:                true
PointCloud input:    /pandar_points_ex
IMU input:           /sensor/imu/data_raw
NMEA input:          /sensor/gnss/nmea_sentence
TF policy:           isolate-all
```

### 4.2 ROSBAG2を実行する

初回はAutoware 1.9.0 base imageを取得し、localizer overlay imageをbuildします。

```bash
ROS_DOMAIN_ID=82 \
  ./script/run_autoware_lsim_docker.sh \
  --bag "$PWD/rosbag/output_pointcloud2" \
  --profile hesai-rosbag23 \
  --tracking-mode scan_to_scan \
  --run-name rosbag2_lsim
```

build時のメモリが不足する場合は`--build-jobs 1`を追加します。通常はbase imageを
意図せず更新しないため`--pull-base`を付けません。

今回の8 CPU / 15.4 GiB環境では、ネットワーク依存の初回base取得に約36分、overlay
buildに約2分かかりました。image取得後の全区間E2Eは各約4分20--25秒です。初回だけ
長時間ログ出力がなくても、Docker pull/build processとディスク増加を確認して待ちます。

### 4.3 ROSBAG3を実行する

同じsourceから続けて実行する場合は、既にbuildしたimageを再利用できます。

```bash
ROS_DOMAIN_ID=83 \
  ./script/run_autoware_lsim_docker.sh \
  --bag "$PWD/rosbag/output_pointcloud3" \
  --profile hesai-rosbag23 \
  --tracking-mode scan_to_scan \
  --run-name rosbag3_lsim \
  --no-build
```

source、Dockerfile、launch、parameterを変更した後は`--no-build`を外してください。

### 4.4 RVizを表示する場合

headless試験を先に完了させてください。表示が必要ならCPU software renderingを使います。

```bash
ROS_DOMAIN_ID=84 \
  ./script/run_autoware_lsim_docker.sh \
  --bag "$PWD/rosbag/output_pointcloud2" \
  --profile hesai-rosbag23 \
  --tracking-mode scan_to_scan \
  --run-name rosbag2_lsim_rviz \
  --no-build \
  --rviz
```

`DISPLAY`とX11 socketが必要です。描画負荷が計測へ影響するため、処理時間の評価には
headless結果を使います。RVizはfull Autoware用の汎用設定ではなく、このprofile専用の
`hesai_rosbag23.rviz`を使います。固定frameは`map`で、deskew後PointCloud、Autoware
kinematic stateの軌跡、TF tree、`base_link`軸を表示します。開始時と再生終了時の両方で
`/rviz2`が生存していなければrunは失敗します。

## 5. Wrapperが行う処理

1. bagと引数を検証し、ROSBAG2/3の固定topic、TF、parameter、GNSS原点を選択する。
2. bag、`build/`、`install/`、`log/`、`test_results/`を除外した小さいDocker build
   contextからoverlay imageをbuildする。
3. Autoware 1.9.0を`use_sim_time=true`かつlocalization-interface-onlyで起動する。
4. launch管理下で3本のstatic TFをpublishする。
5. localizer、GNSS frontend、fusion、Autoware Adapterを起動し、Autoware側の
   `pointcloud_container`を含む必須nodeが揃ったことを確認する。
6. output recorder nodeの起動を確認する。
7. DDS discovery用に2秒待ち、PointCloud2、IMU、NMEAの3 topicだけを再生する。
8. 最初の有効な`/localization/kinematic_state`を受信できなければ失敗させる。
9. bag終了後、recorderをflushしてMCAP metadataを確定し、launchを停止する。

GNSSを使うため自動`/initialpose`は送信しません。複数のGNSS/odometry観測から
初期`map` anchorを決めます。

## 6. 出力

既定の出力rootは`docker_output/`です。上記コマンドでは次の構成になります。

```text
docker_output/
  rosbag2_lsim/
    run.env
    input_bag_info.txt
    launch.log
    replay.log
    record.log
    validation.log
    docker_runtime.txt
    final_nodes.txt
    first_kinematic_state.yaml
    rviz_node_info.txt          # --rviz指定時
    localization_output/
      metadata.yaml
      localization_output_0.mcap
```

同名runが既にある場合はtimestamp suffixを付け、既存結果を上書きしません。
`run.env`には実際に使ったimage、profile、topic、parameter path、clock rate、TF policy、
GNSS recovery modeを保存します。container終了時に出力所有者をホストUID/GIDへ戻します。
`docker_runtime.txt`には実際のlocal image IDとAutoware launch package versionを保存します。
記録終了後はoutput bagを自動検査し、結果を`validation.log`へ保存します。必須topic、
frame、有限値、共分散、時刻、TF、最終diagnosticsのいずれかが不合格ならrun自体も
status 0以外で終了します。

`--rviz`時の`rviz_node_info.txt`は、終了直前まで`/rviz2`が生存した証跡です。
画面のスクリーンショットは自動取得しません。9章の実機試験では、プライバシー保護の
ためデスクトップ全体ではなくRVizウィンドウだけを`rviz_window.png`へ保存しました。

主なAutoware出力は次のとおりです。

| Topic / TF | 型 / 内容 |
|---|---|
| `/localization/kinematic_state` | `nav_msgs/msg/Odometry`, `map -> base_link` |
| `/localization/pose_estimator/pose_with_covariance` | fused pose + covariance |
| `/localization/acceleration` | `base_link`基準の有限差分加速度 |
| `map -> base_link` | Adapterがpublishするdynamic TF |

監査用に`/localization/ekf_odom`、LiDAR odometry、corrected IMU、GNSS fusion input、
GNSS confidence、diagnostics、3本のstatic TFも同じoutput bagへ記録します。

## 7. 合格確認

最低限、次を確認します。

- processがstatus 0で終了し、output MCAPに`metadata.yaml`がある。
- static TFが指定した3本で、同じchild frameを別nodeがpublishしていない。
- deskew診断の`time_field_name=time_stamp`、`used_linear_fallback=false`。
- LiDAR registration受理率が99%以上で、tracking resetが0。
- ROSBAG3の長期遮蔽後に`OUTAGE -> REACQUIRING -> RECOVERING -> TRACKING`へ戻る。
- 最終fusionが`tracking/full_se2`かつposition/yaw fused。
- Autowareの3出力が有限で、frameが`map -> base_link`、stamp逆行が0。
- 最終kinematic stateが最終`/clock`から1秒以内で、active区間の実効rateが45 Hz以上。
- 連続XY stepが0.55 m以下。
- pose covarianceが有限、対称、positive semidefinite。
- Adapterの`rejected_count=0`。同一stamp抑制は
  `duplicate_stamp_drop_count`へ分離されている。

ホストにROS 2 Jazzyとこのworkspaceのbuildがある場合は、記録bagを検査できます。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 tools/analyze_autoware_lsim_output.py \
  docker_output/rosbag2_lsim/localization_output \
  --profile hesai-rosbag23 \
  --tracking-mode scan_to_scan
```

自動実行結果の最低限のgateは次です。

```bash
run=docker_output/rosbag2_lsim
test -f "$run/localization_output/metadata.yaml"
grep -Eq '^Result:[[:space:]]+PASS( \(with WARN\))?$' "$run/validation.log"
grep -Fx /pointcloud_container "$run/final_nodes.txt"
```

RViz runではさらに確認します。

```bash
grep -Fx /rviz2 "$run/final_nodes.txt"
test -s "$run/rviz_node_info.txt"
```

## 8. ROS 2上でのAdapter統合試験

Autoware/Dockerがない開発環境では、同じlocalizer launch、profile TF、parameter、
Adapterを実bagで確認できます。これはAutoware processを起動しないため、Docker lSIM
試験の代替ではありません。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ROS_DOMAIN_ID=90 \
  ./script/run_vehicle_localizer_hesai_nmea_gnss_no_snow_lp.sh \
  --bag rosbag/output_pointcloud2 \
  --lsim-interface-test \
  --output test_results/rosbag2_lsim_interface
```

ROSBAG3ではbagとoutput名を変更します。

## 9. 2026-08-10に実施した試験

### 9.1 Docker + Autoware 1.9.0 E2E

Docker Engine 29.7.2、Compose 5.4.0を使い、公式
`ghcr.io/autowarefoundation/autoware:universe-devel-jazzy-1.9.0`をbaseにしたoverlay
imageで実行しました。container内の`autoware_launch`は0.52.0、ROS distroはJazzyです。
ROSBAG2/3のheadless全区間と、ROSBAG2の実X11 RViz付き全区間がすべてstatus 0で
完了しました。

| clean run | RViz | local image ID（先頭） | 解析message | kinematic state | 実効rate | 最大XY step | 自動acceptance |
|---|---:|---|---:|---:|---:|---:|---|
| `rosbag2_docker_e2e_20260810_retry1` | false | `sha256:8b6d9f68` | 198,636 | 10,175 | 52.236 Hz | 0.400703 m | PASS（35 PASS / 0 FAIL / 1 WARN） |
| `rosbag3_docker_e2e_20260810` | false | `sha256:8b6d9f68` | 202,984 | 10,720 | 54.334 Hz | 0.521073 m | PASS（34 PASS / 0 FAIL / 2 WARN） |
| `rosbag2_docker_rviz_e2e_20260810_retry1` | true | `sha256:18bd1469` | 234,681 | 19,145 | 98.290 Hz | 0.394824 m | PASS（35 PASS / 0 FAIL / 1 WARN） |

RViz runを再buildした契機は、PointCloud2のQoSをJazzy形式で明示する表示設定の変更
です。localizer source、parameter、TF、再生条件は変更していませんが、image IDが異なる
ため証跡上は別imageとして扱います。callback schedulingによって出力rateも異なるため、
この差を性能改善とは解釈せず、性能比較にはheadlessの52.236/54.334 Hzを使います。
すべてのrunで受入下限45 Hzを満たし、stampは厳密単調、最終stateは最終`/clock`まで
到達しました。

| 検査項目 | ROSBAG2 headless | ROSBAG3 headless | ROSBAG2 + RViz |
|---|---:|---:|---:|
| deskew成功 | 4,214/4,215（99.976%） | 4,272/4,273（99.977%） | 4,214/4,215（99.976%） |
| LiDAR registration | 4,188/4,213（99.407%） | 4,251/4,271（99.532%） | 4,188/4,213（99.407%） |
| tracking reset | 0 | 0 | 0 |
| Adapter reject | 0 | 0 | 0 |
| 最終GNSS fusion | tracking / full_se2 | tracking / full_se2 | tracking / full_se2 |
| 必須Autoware node | 全件生存 | 全件生存 | 全件生存、`/rviz2`を含む |

ROSBAG3では2回の遮蔽について
`tracking -> reacquiring -> outage -> reacquiring -> recovering -> tracking`を確認し、
約63秒の長期outage後もfull SE(2)へ復帰しました。WARNはROSBAG2 headlessの古い
publish要求84件、ROSBAG3の15件、RViz runの27件を単調出力guardが抑制した記録です。
公開したEKF/Autoware出力に時刻逆行はありません。ROSBAG3のもう1件は再捕捉付近の
最大XY step 0.521073 mで、受入上限0.55 m以内です。

RViz runでは次も実測しました。

- X11上に1,440 x 900のRVizウィンドウが作成され、OpenGL 4.5で起動した。
- `/localization/points_undistorted`のpublisherとRViz subscriberがともに
  `BEST_EFFORT / VOLATILE`で一致した。
- RVizの`Global Status`と`Deskewed PointCloud Status`がともに`Ok`で、点群を
  31 fpsで実描画した。
- 終了時の`final_nodes.txt`に`/rviz2`があり、`rviz_node_info.txt`も保存された。
- RVizウィンドウ限定の証跡を
  `docker_output/rosbag2_docker_rviz_e2e_20260810_retry1/rviz_window.png`へ保存した。

RViz runのlaunchには、設定を適用して最終QoSへ切り替える間の一過性QoS WARN 2件と、
ROSBAG2入力にある約5 msの逆行IMUを無視したWARN 2件もあります。その後のQoS WARN、
描画ERROR、node crashはなく、出力MCAPの全契約はPASSしています。

保存済みMCAPのartifact IDは次です。再実行時の期待hashではなく、今回の証跡を識別する
ための値です。

| run | MCAP size | SHA-256 |
|---|---:|---|
| ROSBAG2 headless | 142,176,710 B | `8d25a267202ebdf594e1c1c4053fa0cfcb5543abaaeeda22a151feb5ea479773` |
| ROSBAG3 headless | 144,697,292 B | `54ed9f7bbdcb77c00127f467e59aa6b1e6d82edac6dbe4ba6d67c7dcc9195823` |
| ROSBAG2 + RViz | 157,851,491 B | `506a501afb1bdbc3d98f737e247ded6ccd59e640db023c0f518606093c942eb8` |

各clean runには確定済みMCAP、`validation.log`、`docker_runtime.txt`、
`final_nodes.txt`があります。初回の環境・launch問題を検出した途中終了runはclean結果
と分離し、上表には含めていません。

### 9.2 ROS 2 native interface試験

Docker導入前にも、Docker内と同じlocalizer launch、Hesai profile、3本のstatic TF、
100 Hz clock、GNSS設定、AdapterをROS 2 Jazzy上で両bagの全区間に対して実行しました。

| 項目 | ROSBAG2 | ROSBAG3 |
|---|---:|---:|
| runner終了 | status 0 | status 0 |
| output bag全message | 197,041 | 197,349 |
| `/localization/ekf_odom` | 20,123 | 20,353 |
| Autoware kinematic/pose/accel/TF（各） | 10,391 | 9,946 |
| Autoware state実効rate | 53.342 Hz | 50.408 Hz |
| 最大連続XY step | 0.400703 m | 0.510104 m |
| corrected IMU | 42,143 | 42,722 |
| GNSS fusion input | 4,215 | 4,273 |
| static TF | 指定3本のみ | 指定3本のみ |
| 自動acceptance | PASS（35 PASS / 0 FAIL / 1 WARN） | PASS（34 PASS / 0 FAIL / 2 WARN） |
| 最終GNSS fusion | tracking / full_se2 | tracking / full_se2 |

両bagともAutoware interfaceの全出力が有限、共分散が対称PSD、frameが
`map -> base_link`、stampが厳密単調で、pose/TFはkinematic stateと全件一致しました。
Adapterの`rejected_count`は0、deskewは全診断で`time_stamp`を使いlinear fallbackは0、
deskew成功率はROSBAG2が4,214/4,215、ROSBAG3が4,272/4,273です。LiDAR registration
受理率はそれぞれ4,188/4,213（99.407%）と4,251/4,271（99.532%）で、tracking resetは
0です。ROSBAG3では約63秒の長期GNSS outage後も
`OUTAGE -> REACQUIRING -> RECOVERING/full_se2 -> TRACKING`へ復帰しました。

WARNは、fusion内部で古いpublish要求をROSBAG2で3回、ROSBAG3で36回抑制した
scheduler観測値です。公開したEKF/Autoware出力の時刻逆行は両方0です。ROSBAG3の
もう1件は再捕捉付近の最大連続XY step約0.510 mで、既存の実用上限0.55 m以内です。
独立ground truthがないため、これらはinterface・内部整合性試験であり絶対精度試験
ではありません。

保存したclean runは次です。

```text
test_results/rosbag2_lsim_interface_verified_20260810/
test_results/rosbag3_lsim_interface_full_20260810/
```

Docker E2E後にもrepository check、8 reference tests、8 packageのbuild/test、launch
construction、Docker/Compose/wrapperのstatic・dry-run checkを実行し、すべてPASS
しました。運用時の最終gateは、4章のcommand終了statusと各`validation.log`のPASSです。

## 10. 制約

- 両bagに独立したground truth poseがないため、ATE、RPE、絶対位置精度は評価できません。
- GNSSとEKFの差は内部整合性であり、GNSSをfilter入力にも使うため独立精度ではありません。
- static TFは提供値を使っており、bag単独では較正精度を証明できません。
- `scan_to_scan`がROSBAG2/3の受入baselineです。`scan_to_submap`は選択できますが、
  同じbagで別途比較してから採用してください。
- full map、route、planning、controlを動かすには、地図projector、vehicle/sensor model、
  initialization/status interfaceを含む別の統合作業が必要です。

## 11. よくある問題

### `docker: command not found`

Docker EngineとCompose v2をホストへ導入してから再実行します。このリポジトリの
wrapperはDocker daemon自体をインストールしません。

### `/var/run/docker.sock: permission denied`

Docker group追加後のログインセッションへ入り直すか、`newgrp docker`を使います。
一時的には3章の`sg docker -c`形式でも実行できます。`sudo`でwrapperを実行すると
出力やX11認証がroot基準になるため推奨しません。

### `no valid Autoware kinematic state arrived`

次を順に確認します。

```bash
sed -n '1,240p' docker_output/<run-name>/launch.log
sed -n '1,240p' docker_output/<run-name>/replay.log
sed -n '1,240p' docker_output/<run-name>/record.log
```

`--profile hesai-rosbag23`の指定、3本のstatic TF、NMEA入力、deskew成功、GNSS初期化を
確認してください。`--launch-vehicle`や`--already-deskewed`は併用しません。

### buildが重い、またはメモリ不足

```bash
./script/run_autoware_lsim_docker.sh ... --build-jobs 1
```

`.dockerignore`により約13 GBのROSBAG2/3や既存試験結果はbuild contextへ入りません。

### RVizが起動しない

まず`--rviz`なしのheadless runを通してください。その後、ホストの`DISPLAY`とX11利用
権限を確認します。SSH環境やWayland-only環境では追加の画面転送設定が必要です。

点群だけ表示されない場合は実行中containerでQoSを確認します。RViz subscriberと
`imu_undistorter` publisherのReliabilityが両方`BEST_EFFORT`であることが正常です。
起動時にRVizがdefault QoSから設定値へ切り替える間だけ一時警告が出る場合があります。
最終endpointのQoSとRVizの`Deskewed PointCloud Status: Ok`で判定してください。

```bash
ros2 topic info --verbose /localization/points_undistorted
```
