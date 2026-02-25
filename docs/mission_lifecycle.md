# ミッションライフサイクルとモード遷移

## 概要

本ドキュメントは、QGroundControl (QGC) からのミッション送信・ARM・制御実行・ミッション完了・再ARMまでの
一連の流れを、look_ahead_control と mavlink_ros2_bridge の連携を含めて記述する。

## ノード間トピック接続

```
QGroundControl (UDP 14550/14551)
    |
    v
mavlink_ros2_bridge
    | /mav/mission  (Float64MultiArray)  ... ウェイポイント
    | /mav/modes    (MavModes)           ... ARM/DISARM, モード
    v
look_ahead_control
    | /auto_log     (AutoLog)            ... テレメトリ (→ bridge → QGC)
    | /rc_pwm       (UInt16MultiArray)   ... モーター制御 (→ maestro_driver)
    | /cmd_vel      (Twist)              ... シミュレーション用 (→ Gazebo)
    v
モーター / シミュレーション
```

## ArduPilot モード定数

| 定数 | 値 | 意味 |
|---|---|---|
| `ARDUPILOT_AUTO_BASE` | 217 | base_mode: ARM + AUTO |
| `ARDUPILOT_AUTO_CUSTOM` | 10 | custom_mode: AUTO |
| DISARMED base_mode | 89 | base_mode: DISARM + GUIDED |

- `base_mode=217` かつ `custom_mode=10` で「ARM済み・AUTOモード」を意味する
- QGCからのARM/DISARM操作は `COMMAND_LONG cmd=400` (MAV_CMD_COMPONENT_ARM_DISARM)
- QGCからのモード変更は `SET_MODE` または `COMMAND_LONG cmd=176` (MAV_CMD_DO_SET_MODE) の2経路がある

## ミッションライフサイクル

### 全体のステートマシン

```
                    ┌─────────────────────────────────────────────┐
                    |                                             |
                    v                                             |
[1. mission_checker] ──WP受信完了──> [2. mission_start_checker]   |
        ^                                   |                     |
        |                             ARM + AUTO                  |
        |                                   |                     |
        |                                   v                     |
        |                          [3. 制御ループ]                |
        |                                   |                     |
     新ミッション                      全WP到達                   |
     受信(seq=0)                            |                     |
        |                                   v                     |
        |                          [4. ミッション完了]            |
        |                            停止 + disarmトリガー        |
        |                                   |                     |
        |                             disarm確認                  |
        |                                   |                     |
        |                                   v                     |
        └────────────────────── [5. 再ARM待ち] ──────────────────┘
                                   (mission_start_checkerに戻る)
```

### ステート詳細

#### 1. mission_checker (ウェイポイント待ち)

**条件**: `waypoint_total_seq == 0` または `waypoint_total_seq != len(waypoint_seq)`

ノード起動直後はウェイポイントが空のため、ここで待機する。

**ウェイポイント受信の流れ:**

1. QGCでミッション作成 → 「アップロード」ボタン
2. QGC → bridge: `MISSION_COUNT` (ミッションアイテム数)
3. bridge → QGC: `MISSION_REQUEST_INT` (各アイテムを順次要求)
4. QGC → bridge: `MISSION_ITEM_INT` (緯度経度を含むウェイポイント)
5. bridge → `/mav/mission`: `[seq, total_seq, command, lat, lon]`
6. look_ahead_control: `load_waypoint()` で UTM座標に変換して蓄積
7. `seq == 0` 受信時にウェイポイントリストをリセット (新ミッションの上書き)

全ウェイポイント受信完了 → ステート2へ。

**注意**: bridge側で `mission_type` のフィルタリングを行い、fence (type=1) や rally (type=2) の
MISSION_COUNT は無視する。これがないと `mission_total_seq` が 0 に上書きされる。

#### 2. mission_start_checker (ARM + AUTOモード待ち)

**条件**: `base_mode != 217` または `custom_mode != 10`

QGCから以下の操作を行うとこのステートを通過する:

1. モードを AUTO に設定 (custom_mode=10)
2. ARM を実行 (base_mode → 217)

bridge側での処理:
- ARM: `COMMAND_LONG cmd=400, param1=1.0` → `base_mode_ = ARDUPILOT_GUIDED_ARMED (217)`
- AUTO: `SET_MODE` または `COMMAND_LONG cmd=176` → `custom_mode_ = 10`
- 900ms毎のheartbeatで `/mav/modes` にpublish

**注意**: bridge は SET_MODE/DO_SET_MODE 受信時に ARM 状態を保持する。
QGCが base_mode=1 (非ARM) を送信してきても、既に ARM 済みなら base_mode=217 を維持する。

#### 3. 制御ループ (10Hz)

ARM + AUTO が確認されると制御が開始される。各ループイテレーションの処理:

1. **位置取得**: `/gnss/solution` (UTM座標) または `/gnss_odom` (Odometry) から現在位置と姿勢を取得
2. **座標変換**: 前WP→現WPの経路軸に沿った座標系に変換
3. **Look-ahead制御**: ルックアヘッド距離に基づきベアリング角を計算
4. **前後判定**: 前進/後退のどちらがヨー誤差が小さいかで走行方向を決定
5. **PID計算**: `pid = Kp * steering_ang - Kcte * cross_track_error`
6. **出力**: `/rc_pwm` (PWM) と `/cmd_vel` (速度) を publish
7. **テレメトリ**: `/auto_log` に制御状態を publish (bridge経由でQGCに送信)
8. **WP到達判定**: 変換座標上で `(wp_x_tf - own_x_tf) < wp_arrival_dist` なら次WPへ

#### 4. ミッション完了 (disarmシーケンス)

**条件**: `seq >= len(waypoint_x)` (全ウェイポイントを通過)

以下の処理を disarm が確認されるまで繰り返す:

1. **停止PWM送信**: `[pwm_center, pwm_center]` を `/rc_pwm` に publish
2. **停止cmd_vel**: `linear.x=0, angular.z=0` を `/cmd_vel` に publish
3. **disarmトリガー**: `/auto_log` に `waypoint_seq = waypoint_total_seq + 1` を publish

bridge側の処理:
- heartbeatタイマーで `current_seq_ > mission_total_seq_` を検出
- `base_mode_` を 89 (DISARMED) に設定、`mission_start_` を false に設定
- `/mav/modes` で disarm 状態を publish → QGC上でもDISARM表示

look_ahead側の処理:
- `/mav/modes` で `base_mode != 217` を確認 → disarm確定
- `waypoint_seq = 0` の auto_log を送信して bridge の `current_seq_` をリセット
- `seq = 1` にリセットして制御ループの先頭に戻る

#### 5. 再ARM待ち (mission_start_checkerに戻る)

disarm確認後、ステート2 (mission_start_checker) に戻る。

- **ウェイポイントは保持**される (リセットされない)
- QGCから再ARM + AUTOモード設定 → ステート3に遷移し、WP1から再走行

**ウェイポイントの上書き**: 再ARM前に QGC から新ミッションをアップロードすると、
`load_waypoint()` の `seq == 0` 判定でウェイポイントリストがリセットされ、新しいミッションに置換される。

## シーケンス図

```
QGC                     bridge                  look_ahead
 |                        |                        |
 |-- MISSION_COUNT ------>|                        |  [mission_checker]
 |<- MISSION_REQUEST_INT -|                        |
 |-- MISSION_ITEM_INT --->|-- /mav/mission ------->|  WP蓄積
 |   (繰り返し)           |   (繰り返し)           |
 |                        |                        |  全WP受信完了
 |                        |                        |  [mission_start_checker]
 |-- DO_SET_MODE(AUTO) -->|                        |
 |-- ARM (cmd=400) ------>|-- /mav/modes --------->|  base=217, custom=10
 |                        |                        |
 |                        |                        |  [制御ループ開始]
 |                        |<- /auto_log -----------|  seq=1,2,3...
 |<- GPS/ATTITUDE/etc. ---|                        |
 |                        |                        |  --> /rc_pwm, /cmd_vel
 |                        |                        |
 |                        |                        |  全WP到達
 |                        |                        |  [ミッション完了]
 |                        |<- /auto_log -----------|  seq=total+1
 |                        |  (cur > total)         |
 |                        |  AUTO-DISARM           |
 |                        |-- /mav/modes --------->|  base=89 (DISARM)
 |                        |                        |
 |                        |<- /auto_log -----------|  seq=0 (リセット)
 |                        |                        |  [mission_start_checker]
 |                        |                        |
 |-- ARM (cmd=400) ------>|-- /mav/modes --------->|  base=217, custom=10
 |                        |                        |  [制御ループ再開 seq=1]
```

## bridge の current_seq リセット問題

ミッション完了時に bridge の `current_seq_` が `waypoint_total_seq + 1` に設定される。
この値がリセットされないまま再ARMすると、heartbeatで即座に `current_seq_ > mission_total_seq_` と
判定され、再disarmが発生する。

対策として、look_ahead_control は disarm 確認後に `auto_log.waypoint_seq = 0` を送信し、
bridge の `current_seq_` を 0 にリセットする。

## パラメータ同期

QGCからのパラメータ変更は以下の経路で look_ahead_control に反映される:

```
QGC (PARAM_SET)
    |
    v
mavlink_ros2_bridge (ROS2パラメータ更新)
    |
    +-- ~/.ros/mavlink_bridge_params.yaml に永続化
    |
    +-- /parameter_events に変更をpublish
         |
         v
look_ahead_control (param_event_callback)
    |
    +-- 起動時: _load_bridge_params() で YAML から読み込み
    +-- 実行時: /parameter_events を購読して動的反映
```

## トラブルシューティング

| 症状 | 原因 | 対処 |
|---|---|---|
| mission_checker で止まる | ウェイポイント未受信 / total_seq と受信数の不一致 | QGCからミッション再アップロード |
| mission_start_checker で止まる | ARM/AUTOモードでない | QGCでAUTOモード設定 + ARM実行 |
| ARM直後にDISARMされる | bridge の current_seq_ が前回値のまま | look_ahead 再起動、または新ミッション送信 |
| custom_mode が 10 でない | QGCで AUTO 以外のモード (Guided=15等) | QGCでモードをAUTOに変更 |
| 制御が始まらない (位置データなし) | GNSS/fake_gnss 未起動 | odom_source に対応するノードを起動 |
| ミッション完了後に再走行しない | disarm後の再ARM手順 | QGCから AUTO + ARM を再度実行 |
