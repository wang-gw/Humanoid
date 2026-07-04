# 97. Unload + Small IK Lift Probe

## 목적

이 단계의 목적은 이전 one-leg support best controller에 작은 오른발 IK lift를 결합해서 오른발 무접촉 시간을 늘릴 수 있는지 확인하는 것이다.

이전 단계 결과:

- 오른발 하중 `<= 5 N` 시간은 최대 `0.528 s`
- 오른발 접촉 `= 0` 시간은 최대 `0.330 s`
- roll은 `0.12 rad` 근처까지 안정적으로 유지
- 최종 gate `right force <= 5 N`, `right contacts = 0`, `roll <= 0.12 rad` 동시 `0.5 s`는 실패

이번 단계의 가설:

- 하중은 충분히 줄어드는 순간이 있으므로, 작은 foot-frame IK lift를 추가하면 오른발 접촉 해제 시간이 `0.5 s` 이상으로 늘어날 수 있다.

## 사용 모델

- `envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml`

기준 pose:

- `configs/weight_shift_left065_contact_constrained_multipoint.json`

IK lift pose:

- `configs/right_foot_lift002_ik_stl_footprint_contact.json`
- `configs/right_foot_lift005_ik_stl_footprint_contact.json`

## 추가 스크립트

- `scripts/render_unload_lift_sequence.py`

기존 `render_right_foot_lift_sequence.py`는 force feedback을 모든 actuator에 동일하게 더한다. 하지만 one-leg support best controller는 다음 조건을 사용했다.

- `force_role = both`
- `force_on = all`
- `force_side_mode = opposite`

따라서 같은 controller를 재현하기 위해 별도 렌더러를 추가했다.

## Controller 기준

이전 단계 narrow best controller:

| 항목 | 값 |
|---|---:|
| target left ratio | 0.94 |
| kforce | 10.0 |
| force role | both |
| force on | all |
| force side mode | opposite |
| joint Kp / Kd | 20 / 12 |
| torque limit | 30 Nm |
| kp_att / kd_att / kcom | 0 / 1 / 1 |

## 2 mm IK Lift 결합

실행:

```bash
MUJOCO_GL=egl python3 scripts/render_unload_lift_sequence.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --poses configs/weight_shift_left065_contact_constrained_multipoint.json configs/right_foot_lift002_ik_stl_footprint_contact.json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --out-dir outputs/analysis/render_unload_lift002_ik_narrow_controller_stl_footprint_contact_8s \
  --duration 8.0 --ramp 2.0 --hold 2.0 \
  --target-left-ratio 0.94 \
  --joint-kp 20 --joint-kd 12 --torque-limit 30 \
  --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 \
  --kforce 10 --force-sign 1 --force-role both --force-on all --force-side-mode opposite \
  --fps 15 --azimuth 135 --elevation -12 --distance 1.45
```

산출물:

- 영상:
  - `outputs/analysis/render_unload_lift002_ik_narrow_controller_stl_footprint_contact_8s/unload_lift_render.mp4`
- timeline:
  - `outputs/analysis/render_unload_lift002_ik_narrow_controller_stl_footprint_contact_8s/unload_lift_timeline.csv`

결과:

| 항목 | 값 |
|---|---:|
| gate time, roll <= 0.12 | 0.330 s |
| gate time, roll <= 0.18 | 0.330 s |
| right force <= 5 N | 0.594 s |
| right contacts = 0 | 0.330 s |
| right contacts <= 1 | 2.310 s |
| clearance > 0 | 0.330 s |
| clearance > 2 mm | 0.132 s |
| max clearance | 9.89 mm |
| final clearance | -0.079 mm |
| max roll | 0.177 rad |
| max pitch | 0.094 rad |
| max torque | 11.82 Nm |
| final right force | 17.93 N |
| final right contacts | 12 |

## 5 mm IK Lift 결합

실행:

```bash
MUJOCO_GL=egl python3 scripts/render_unload_lift_sequence.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --poses configs/weight_shift_left065_contact_constrained_multipoint.json configs/right_foot_lift005_ik_stl_footprint_contact.json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --out-dir outputs/analysis/render_unload_lift005_ik_narrow_controller_stl_footprint_contact_8s \
  --duration 8.0 --ramp 2.0 --hold 2.0 \
  --target-left-ratio 0.94 \
  --joint-kp 20 --joint-kd 12 --torque-limit 30 \
  --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 \
  --kforce 10 --force-sign 1 --force-role both --force-on all --force-side-mode opposite \
  --fps 15 --azimuth 135 --elevation -12 --distance 1.45
```

산출물:

- 영상:
  - `outputs/analysis/render_unload_lift005_ik_narrow_controller_stl_footprint_contact_8s/unload_lift_render.mp4`
- timeline:
  - `outputs/analysis/render_unload_lift005_ik_narrow_controller_stl_footprint_contact_8s/unload_lift_timeline.csv`

결과:

| 항목 | 값 |
|---|---:|
| gate time, roll <= 0.12 | 0.330 s |
| gate time, roll <= 0.18 | 0.330 s |
| right force <= 5 N | 0.528 s |
| right contacts = 0 | 0.330 s |
| right contacts <= 1 | 2.508 s |
| clearance > 0 | 0.330 s |
| clearance > 2 mm | 0.132 s |
| max clearance | 9.89 mm |
| final clearance | -0.108 mm |
| max roll | 0.207 rad |
| max pitch | 0.130 rad |
| max torque | 11.82 Nm |
| final right force | 18.95 N |
| final right contacts | 11 |

## 2 mm IK Lift Slow Trajectory

2 mm lift가 과도하게 빠른 ramp 때문에 실패하는지 확인하기 위해 ramp를 `4 s`로 늘린 12초 trajectory를 추가 확인했다.

실행:

```bash
MUJOCO_GL=egl python3 scripts/render_unload_lift_sequence.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --poses configs/weight_shift_left065_contact_constrained_multipoint.json configs/right_foot_lift002_ik_stl_footprint_contact.json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --out-dir outputs/analysis/render_unload_lift002_ik_narrow_controller_slow_stl_footprint_contact_12s \
  --duration 12.0 --ramp 4.0 --hold 2.0 \
  --target-left-ratio 0.94 \
  --joint-kp 20 --joint-kd 12 --torque-limit 30 \
  --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 \
  --kforce 10 --force-sign 1 --force-role both --force-on all --force-side-mode opposite \
  --fps 15 --azimuth 135 --elevation -12 --distance 1.45
```

산출물:

- 영상:
  - `outputs/analysis/render_unload_lift002_ik_narrow_controller_slow_stl_footprint_contact_12s/unload_lift_render.mp4`
- timeline:
  - `outputs/analysis/render_unload_lift002_ik_narrow_controller_slow_stl_footprint_contact_12s/unload_lift_timeline.csv`

결과:

| 항목 | 값 |
|---|---:|
| gate time, roll <= 0.12 | 0.330 s |
| right force <= 5 N | 0.528 s |
| right contacts = 0 | 0.330 s |
| right contacts <= 1 | 3.366 s |
| clearance > 0 | 0.330 s |
| clearance > 2 mm | 0.132 s |
| max clearance | 9.89 mm |
| final clearance | -0.415 mm |
| max roll | 1.009 rad |
| max pitch | 0.096 rad |
| max torque | 11.82 Nm |
| final right force | 21.40 N |
| final right contacts | 1 |

느린 trajectory는 오른발 접촉 수 `<= 1` 시간은 늘렸지만, roll이 크게 붕괴했다. 따라서 단순히 ramp를 느리게 하는 것은 해결책이 아니다.

## 비교

| 항목 | one-leg only | unload + 2 mm IK | unload + 5 mm IK | slow 2 mm IK |
|---|---:|---:|---:|---:|
| gate time | 0.330 s | 0.330 s | 0.330 s | 0.330 s |
| right force <= 5 N | 0.528 s | 0.594 s | 0.528 s | 0.528 s |
| right contacts = 0 | 0.330 s | 0.330 s | 0.330 s | 0.330 s |
| right contacts <= 1 | 2.244 s | 2.310 s | 2.508 s | 3.366 s |
| clearance > 2 mm | - | 0.132 s | 0.132 s | 0.132 s |
| max roll | 0.117 rad | 0.177 rad | 0.207 rad | 1.009 rad |
| max torque | 11.82 Nm | 11.82 Nm | 11.82 Nm | 11.82 Nm |

## 판단

작은 IK lift를 결합해도 핵심 gate는 개선되지 않았다.

확인된 사실:

- 오른발 하중을 줄이는 능력은 있다.
- 순간적으로 오른발 접촉을 완전히 끊는 것도 가능하다.
- 하지만 접촉 완전 해제 시간은 `0.330 s` 근처에서 더 늘지 않는다.
- lift를 더 크게 넣으면 roll 안정성이 나빠진다.
- ramp를 더 느리게 해도 접촉 해제 시간은 늘지 않고, 장기 roll collapse가 발생한다.

따라서 현재 병목은 단순한 발끝 lift height 문제가 아니다. 더 근본적으로는 왼발 support polygon 안에서 COM/base를 유지하면서 오른발 접촉을 끊는 closed-loop balance가 부족하거나, 하드웨어 형상 자체의 한발 지지 안정 여유가 작을 가능성이 있다.

## 현재 Gate 상태

| 단계 | 결과 |
|---|---|
| standing | 통과 |
| weight shift | 통과 |
| right unload | 통과 |
| transient toe-off | 통과 |
| foot-frame IK pose generation | 통과 |
| unload + small IK lift | 실패 |
| one-leg support 0.5초 | 실패 |
| 2 mm clearance 0.5초 유지 | 실패 |

## 다음 조치

다음 단계는 controller 튜닝만 계속하기보다 설계 검증 관점의 민감도 분석으로 넘어가는 것이 맞다.

권장 probe:

1. 왼발 support polygon을 가상으로 키웠을 때 one-leg support gate가 통과되는지 확인한다.
2. 상체/base COM을 좌측 지지발 쪽으로 가상 이동했을 때 gate가 개선되는지 확인한다.
3. 발목 roll/pitch 축 위치 또는 발바닥 폭/길이 중 어떤 설계 변수가 gate에 가장 큰 영향을 주는지 sweep한다.
4. 가상 설계 변경에서 gate가 통과되면 실제 CAD 수정 후보로 기록한다.
5. 가상 변경에서도 gate가 안 되면 제어 구조를 RL 또는 MPC 성격으로 바꿔야 한다.
