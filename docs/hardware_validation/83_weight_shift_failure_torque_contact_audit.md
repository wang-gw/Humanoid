# Weight Shift Failure Torque/Contact Audit

## 목적

82번에서 `standing -> 0.62`는 성공했지만, `standing -> 0.62 -> 0.65`는 실패했다.

이번 단계에서는 실패 원인을 분리하기 위해 같은 sequence를 렌더링 없이 다시 재생하면서 다음 값을 2 ms 단위로 기록했다.

- left/right normal force ratio
- foot pad별 contact normal force
- roll/pitch/yaw
- COM error
- actuator별 PD torque, stabilizer torque, control torque

## 추가한 스크립트

```text
scripts/audit_pose_sequence_torque_contact.py
```

이 스크립트는 `render_pose_sequence.py`와 같은 target interpolation을 사용하지만, 영상 대신 torque/contact CSV를 저장한다.

## 감사 대상

실패 sequence:

```text
standing -> transition-aware 0.62 -> static 0.65
```

입력 pose:

```text
configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json
configs/weight_shift_left062_transition_aware_multipoint.json
configs/weight_shift_left065_pose_multipoint.json
```

실행:

```bash
python3 scripts/audit_pose_sequence_torque_contact.py --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics_multipoint_fullheight_contact.xml --poses configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json configs/weight_shift_left062_transition_aware_multipoint.json configs/weight_shift_left065_pose_multipoint.json --out-dir outputs/analysis/audit_sequence_torque_contact_standing_062_065_multipoint --ramp 4.0 --hold 4.0 --joint-kp 20 --joint-kd 12 --torque-limit 30 --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 --log-every 1
```

## 실패 발생 시점

roll threshold 최초 도달 시점:

| threshold | time |
| --- | ---: |
| `abs(roll) >= 0.1 rad` | `7.366 s` |
| `abs(roll) >= 0.25 rad` | `14.394 s` |
| `abs(roll) >= 0.5 rad` | `14.418 s` |
| `abs(roll) >= 1.0 rad` | `14.444 s` |
| `abs(roll) >= 1.5 rad` | `14.466 s` |
| `abs(roll) >= 2.0 rad` | `14.494 s` |

해석:

- `0.62` 단계에서 작은 roll offset은 존재하지만 안정적이다.
- 실제 붕괴는 `0.65` phase 후반, 약 `14.39 s`부터 급격히 시작된다.
- `0.25 rad -> 2.0 rad`까지 약 `0.10 s` 안에 진행된다.

## 성공 경로와 실패 경로 비교

성공 경로:

```text
standing -> transition-aware 0.62
```

실패 경로:

```text
standing -> transition-aware 0.62 -> static 0.65
```

### 성공 경로 8~12초

| 항목 | 값 |
| --- | ---: |
| left force ratio avg | `0.615365` |
| left force ratio range | `0.613137 ~ 0.617125` |
| roll avg | `0.104092 rad` |
| qvel avg | `0.009194` |
| normal force avg | `90.363911 N` |
| left contacts avg | `16.0` |
| right contacts avg | `4.0` |
| COM err Y avg | `0.158251` |
| max torque | 약 `3.76 Nm` |

성공 경로는 접촉점 수와 하중비가 거의 고정되어 있다.

### 실패 경로 12~14초

| 항목 | 값 |
| --- | ---: |
| left force ratio avg | `0.419236` |
| left force ratio range | `0.173381 ~ 0.533906` |
| roll range | `-0.083003 ~ 0.035696 rad` |
| pitch min | `-0.311169 rad` |
| qvel max | `1.151807` |
| left contacts avg | `3.069` |
| right contacts avg | `2.834` |
| COM err Y avg | `0.704906` |
| right hip roll ctrl avg | `-2.692761 Nm` |
| right hip pitch ctrl avg | `-3.703225 Nm` |

여기서 이미 문제가 시작된다.

중요한 점은 `0.65`로 가는 중에 왼발 하중이 늘어나는 것이 아니라 오히려 줄어든다는 것이다. 성공 경로에서는 left contacts가 `16`개 유지되는데, 실패 경로에서는 평균 `3`개 수준으로 줄어든다.

### 실패 경로 14.0~14.35초

| 항목 | 값 |
| --- | ---: |
| left force ratio avg | `0.191800` |
| left force ratio min/max | `0.166981 / 0.375699` |
| pitch min | `-1.026744 rad` |
| qvel max | `4.284306` |
| normal force avg | `71.528446 N` |
| left contacts avg | `1.12` |
| right contacts avg | `1.15` |
| COM err Y avg | `2.207426` |
| roll term avg | `1.635917` |
| pitch term avg | `-2.892403` |
| right hip roll ctrl avg | `-8.568466 Nm` |
| right hip roll ctrl max abs | `12.792166 Nm` |
| right hip pitch ctrl avg | `-5.107368 Nm` |

이 구간은 붕괴 직전이다.

관찰:

- 왼발 접촉이 거의 사라진다.
- COM lateral error가 크게 증가한다.
- right hip roll 역할 joint의 control torque가 가장 크게 증가한다.
- 그러나 torque saturation은 없다.

### 실패 경로 14.35~14.50초

| 항목 | 값 |
| --- | ---: |
| left force ratio avg | `0.265274` |
| roll range | `0.031526 ~ 2.058138 rad` |
| pitch avg | `-1.234705 rad` |
| yaw avg | `-1.737075 rad` |
| qvel max | `6.575554` |
| normal force max | `183.991030 N` |
| COM err Y avg | `3.787087` |
| right hip roll ctrl avg | `-13.269410 Nm` |
| right hip roll ctrl max abs | `17.123201 Nm` |
| left hip pitch ctrl avg | `-6.907240 Nm` |

여기서 실제 roll collapse가 발생한다.

## Pad별 접촉력

### 8~12초

주요 pad 평균 normal force:

| pad | avg force |
| --- | ---: |
| `foot_L_1_sole_pad_rear_left` | `19.901 N` |
| `foot_R_v1_1_sole_pad_rear_left` | `19.131 N` |
| `foot_R_v1_1_sole_pad_rear_right` | `17.593 N` |
| `foot_L_1_sole_pad_front_left` | `16.594 N` |

아직 좌우 모두 여러 pad에 접촉이 분산되어 있다.

### 12~14초

| pad | avg force |
| --- | ---: |
| `foot_R_v1_1_sole_pad_rear_left` | `44.793 N` |
| `foot_L_1_sole_pad_rear_left` | `32.101 N` |
| `foot_R_v1_1_sole_pad_rear_right` | `9.443 N` |
| `foot_L_1_sole_pad_front_left` | `6.581 N` |

오른발 rear-left pad로 하중이 몰리기 시작한다.

### 14.0~14.35초

| pad | avg force |
| --- | ---: |
| `foot_R_v1_1_sole_pad_rear_left` | `56.107 N` |
| `foot_R_v1_1_sole_pad_rear_right` | `31.051 N` |
| `foot_L_1_sole_pad_rear_right` | `17.712 N` |
| `foot_L_1_sole_pad_rear_left` | `12.271 N` |

붕괴 직전에는 하중이 오른발 rear pad 쪽으로 더 강하게 이동한다.

이것은 목표와 반대 방향이다. `0.65` 목표는 왼발 하중 증가인데, 실제 접촉은 오른발 rear 쪽으로 몰린다.

## 그래프

![sequence failure audit](../../outputs/analysis/audit_sequence_torque_contact_standing_062_065_multipoint/sequence_failure_audit_plot.png)

## 핵심 결론

`0.62 -> 0.65` 실패는 모터 torque limit에 걸려서 생긴 문제가 아니다.

확인된 사실:

1. 실패 시 torque saturation은 없다.
2. `0.65`로 전이하는 중 왼발 하중이 증가하지 않고 오히려 감소한다.
3. 왼발 contact pad 수가 `16`개 수준에서 `3`개 이하로 줄어든다.
4. 하중은 오른발 rear pad 쪽으로 몰린다.
5. right hip roll 역할 joint torque가 가장 크게 증가한다.
6. 붕괴는 roll보다 pitch/contact 변화가 먼저 커지고, 이후 roll이 약 `0.1 s` 안에 급격히 커지는 형태다.

따라서 현재 병목은 다음 중 하나에 가깝다.

- `0.65` 목표 pose가 실제로는 왼발 하중 이동 pose가 아니라, 접촉을 잃고 오른발 rear로 굴러가는 pose다.
- 현재 controller의 COM error 기반 보정이 left/right force ratio를 직접 제어하지 못한다.
- hip/ankle 역할 분배가 lateral force transfer를 만들지 못하고, right hip roll 쪽에 큰 보정 torque가 몰린다.

## 다음 조치

다음 단계에서는 lateral controller를 바꿔야 한다.

추천:

1. `left_force_ratio`를 직접 feedback으로 쓰는 controller를 만든다.
2. 목표 force ratio와 실제 force ratio의 차이를 hip/ankle roll 역할 actuator에 넣는다.
3. 우선 작은 목표인 `0.62 -> 0.65`만 테스트한다.
4. 이 방식으로도 하중이 오른발 rear로 몰리면, pose family 또는 발/ankle 기구학 자체가 목표 하중 이동에 불리한 것이다.

## 산출물

- torque/contact CSV: `/home/king0519/projects/Humanoid/outputs/analysis/audit_sequence_torque_contact_standing_062_065_multipoint/sequence_torque_contact_timeline.csv`
- pad contact CSV: `/home/king0519/projects/Humanoid/outputs/analysis/audit_sequence_torque_contact_standing_062_065_multipoint/sequence_contact_force_by_geom.csv`
- summary JSON: `/home/king0519/projects/Humanoid/outputs/analysis/audit_sequence_torque_contact_standing_062_065_multipoint/sequence_torque_contact_summary.json`
- audit plot: `/home/king0519/projects/Humanoid/outputs/analysis/audit_sequence_torque_contact_standing_062_065_multipoint/sequence_failure_audit_plot.png`
- 성공 `0.62` 비교 CSV: `/home/king0519/projects/Humanoid/outputs/analysis/audit_sequence_torque_contact_standing_062_transition_aware_multipoint/sequence_torque_contact_timeline.csv`
