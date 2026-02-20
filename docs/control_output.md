# 制御出力とパラメータの関係

## 出力の構造

本ノードはRC PWMを主出力とし、cmd_velはPWMから逆算される副出力である。
`driver_mix` パラメータにより、ノード側で差動ミキシングを行うか、
モータードライバに任せるかを切り替えられる。

```
制御ロジック (steering_ang, translation, pid)
    |
    v
throttle, steering 算出
    |
    +-- driver_mix=0 (differential) --> ch1=left,     ch2=right
    |
    +-- driver_mix=1 (passthrough)  --> ch1=throttle, ch2=steering
    |
    v
PWMマッピング + クランプ
    |
    +---> /rc_pwm  [ch1_pwm, ch2_pwm]      ... 主出力
    |
    +---> /cmd_vel  (linear.x, angular.z)   ... 副出力 (PWMから逆算)
```

## パラメータ一覧

全パラメータはdouble型。MAVLink GCS (QGroundControl) からfloat値で設定可能。
mavlink_ros2_bridge の paramEntries に登録済み。

### 制御パラメータ

| パラメータ | デフォルト | 説明 |
|---|---|---|
| `Kp` | 0.0 | 比例ゲイン。steering_ang(度) に乗算してpid値を生成 |
| `Kcte` | 0.0 | クロストラックエラーゲイン。横偏差(m)に乗算 |
| `look_ahead` | 0.0 | ルックアヘッド距離 (m) |
| `pivot_threshold` | 40.0 | ピボットターンに切り替わるヨー誤差の閾値 (度) |
| `cte_threshold` | 0.1 | CTE補正が有効になる横偏差の閾値 (m) |
| `wp_arrival_dist` | 0.1 | ウェイポイント到達と判定する距離 (m) |
| `wp_skip_dist` | 0.8 | これより近い連続ウェイポイントをスキップする距離 (m) |
| `throttle_scale` | 0.5 | 直進時のモーター出力 (0.0 ~ 1.0) |
| `pivot_scale` | 0.5 | ピボットターン時のモーター出力 (0.0 ~ 1.0) |

### モード・PWMパラメータ

| パラメータ | デフォルト | 説明 |
|---|---|---|
| `driver_mix` | 0.0 | 0.0=differential (ノードがミキシング), 1.0=passthrough (ドライバがミキシング) |
| `pwm_center` | 1500.0 | モーター停止のPWM中心値 (us) |
| `pwm_range` | 500.0 | 中心からの最大偏差 (us) |
| `pwm_min` | 1000.0 | PWM安全下限 (us) |
| `pwm_max` | 2000.0 | PWM安全上限 (us) |

## ミキシングモード

### differential モード (driver_mix=0.0)

ノードが差動ミキシングを行い、左右モーターのPWMを出力する。
モータードライバがミキシング機能を持たない場合に使用。

```
/rc_pwm = [left_pwm, right_pwm]
```

#### チャンネル値の計算

通常走行時 (|steering_ang| <= 40度):

```
throttle = throttle_scale * translation
ch1 (left)  = throttle - pid
ch2 (right) = throttle + pid
```

ピボットターン時 (|steering_ang| > 40度):

```
左旋回: ch1 = -pivot_scale,  ch2 = pivot_scale
右旋回: ch1 = pivot_scale,   ch2 = -pivot_scale
```

#### cmd_vel の逆算

```
ch1_n     = (ch1_pwm - pwm_center) / pwm_range
ch2_n     = (ch2_pwm - pwm_center) / pwm_range
linear.x  = (ch1_n + ch2_n) / 2.0
angular.z = (ch2_n - ch1_n) / 2.0
```

### passthrough モード (driver_mix=1.0)

ノードはミキシングせず、スロットルとステアリングを個別にPWM出力する。
モータードライバ側にミキシング機能がある場合に使用。

```
/rc_pwm = [throttle_pwm, steering_pwm]
```

#### チャンネル値の計算

通常走行時:

```
ch1 (throttle) = throttle_scale * translation
ch2 (steering) = pid
```

ピボットターン時:

```
ch1 (throttle) = 0.0
ch2 (steering) = +/-pivot_scale
```

#### cmd_vel の逆算

```
linear.x  = (ch1_pwm - pwm_center) / pwm_range
angular.z = (ch2_pwm - pwm_center) / pwm_range
```

## PWMマッピング (共通)

```
ch1_pwm = pwm_center + ch1 * pwm_range
ch2_pwm = pwm_center + ch2 * pwm_range
```

ch1, ch2 はそれぞれ [-1.0, 1.0] にクランプされた後にマッピングされる。
さらに [pwm_min, pwm_max] で安全クランプされる。

### 例 (デフォルト設定: pwm_center=1500, pwm_range=500)

| ch値 | PWM |
|---|---|
| -1.0 | 1000 |
| -0.5 | 1250 |
| 0.0 | 1500 |
| 0.5 | 1750 |
| 1.0 | 2000 |

## cmd_vel のレンジ

### differential モード

| 状態 | linear.x | angular.z |
|---|---|---|
| 直進 | +throttle_scale | 0.0 |
| 後退 | -throttle_scale | 0.0 |
| ピボットターン | 0.0 | +/-pivot_scale |
| 通常走行 + ステアリング | throttle_scale寄り (pid増で減少) | pid依存 |

throttle_scale が linear.x の最大値を直接決定する。

### passthrough モード

| 状態 | linear.x | angular.z |
|---|---|---|
| 直進 | throttle_scale | 0.0 |
| 後退 | -throttle_scale | 0.0 |
| ピボットターン | 0.0 | +/-pivot_scale |
| 通常走行 + ステアリング | throttle_scale | pid ([-1,1] にクランプ) |

passthrough では linear.x はステアリングの影響を受けない。

### 数値例 (differential, throttle_scale=0.5, pivot_scale=0.5)

| シナリオ | pid | linear.x | angular.z |
|---|---|---|---|
| 直進 | 0.0 | 0.50 | 0.00 |
| 後退 | 0.0 | -0.50 | 0.00 |
| ピボットターン | - | 0.00 | 0.50 |
| 軽いステアリング | 0.1 | 0.50 | 0.10 |
| 中程度ステアリング | 0.3 | 0.50 | 0.30 |
| 強いステアリング | 0.6 | 0.45 | 0.55 |

強いステアリング時、左モーターが負 (ch1 = throttle - pid = -0.1) になり、
linear.x が throttle_scale より小さくなる。これは片輪が逆転しているためである。

## チューニング指針

### cmd_vel を -0.6 ~ 0.6 に収めたい場合

- `throttle_scale = 0.6` に設定 → linear.x の最大が 0.6
- `pivot_scale = 0.5` に設定 → ピボット時の angular.z が 0.5
- `Kp` を小さめに設定 (0.01 ~ 0.02 程度) → 通常走行中の angular.z を制御

### Kp の目安

pid = Kp * steering_ang であり、steering_ang の最大は YAW_TOLERANCE = 40度。

| Kp | 最大pid (40度時) | throttle_scale=0.5 での挙動 |
|---|---|---|
| 0.005 | 0.2 | 穏やかなステアリング、飽和なし |
| 0.01 | 0.4 | 中程度のステアリング、飽和なし |
| 0.02 | 0.8 | 強いステアリング、片輪逆転あり |
| 0.05 | 2.0 | 両輪飽和、ほぼピボットに近い動作 |

### PWM レンジの調整

実機のESC/モータードライバに合わせて pwm_center, pwm_range, pwm_min, pwm_max を設定する。
制御ロジック側 (throttle_scale, Kp等) は正規化された [-1, 1] 空間で動作するため、
PWMパラメータを変更しても制御挙動は変わらない。

### driver_mix の選択

| ドライバの種類 | driver_mix | /rc_pwm の意味 |
|---|---|---|
| 左右独立モータードライバ | 0.0 | [left_pwm, right_pwm] |
| ミキシング内蔵ドライバ | 1.0 | [throttle_pwm, steering_pwm] |
