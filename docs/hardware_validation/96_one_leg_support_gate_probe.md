# 96. One-leg Support Gate Probe

## 목적

이 단계의 목적은 오른발을 들어 올리는 trajectory를 더 탐색하기 전에, 왼발 단독 지지 상태가 가능한지 분리해서 확인하는 것이다.

이전 단계에서 foot-frame IK는 성공했지만, 동역학 시뮬레이션에서는 오른발 clearance가 약 `0.132 s`만 유지됐다. 따라서 이번에는 lift height보다 먼저 다음 gate를 확인했다.

Gate:

- 오른발 하중 `<= 5 N`
- 오른발 접촉 수 `= 0`
- base roll `<= 0.12 rad`
- 위 조건을 동시에 `>= 0.5 s` 유지

## 사용 모델

- `envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml`

기준 pose:

- `configs/weight_shift_left065_contact_constrained_multipoint.json`

## 추가 스크립트

- `scripts/search_one_leg_support_gate.py`

이 스크립트는 force-ratio feedback controller 후보를 평가한다.

평가 대상:

- target left force ratio
- force feedback gain
- force feedback 적용 role
- force feedback 적용 actuator group
- 좌우 actuator 적용 방향

점수 기준:

- gate 유지시간 증가
- 오른발 low-force 시간 증가
- 오른발 no-contact 시간 증가
- roll gate 초과 패널티
- torque saturation 패널티

## Fast Sweep

실행:

```bash
python3 scripts/search_one_leg_support_gate.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --pose configs/weight_shift_left065_contact_constrained_multipoint.json \
  --out-dir outputs/analysis/one_leg_support_gate_fast_stl_footprint_contact \
  --duration 6.0 --target-gate-time 0.5 \
  --right-force-gate 5.0 --right-contact-gate 0.0 --max-roll-gate 0.12 \
  --joint-kp 20 --joint-kd 12 --torque-limit 30 \
  --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 \
  --force-sign 1 --fast
```

산출물:

- summary:
  - `outputs/analysis/one_leg_support_gate_fast_stl_footprint_contact/one_leg_support_gate_summary.json`
- candidates:
  - `outputs/analysis/one_leg_support_gate_fast_stl_footprint_contact/one_leg_support_gate_candidates.csv`

Best candidate:

| 항목 | 값 |
|---|---:|
| target left ratio | 0.90 |
| kforce | 12.0 |
| force role | both |
| force on | all |
| force side mode | opposite |
| internal gate time | 0.332 s |
| low force time | 0.364 s |
| no contact time | 0.332 s |
| max roll | 0.100 rad |
| max pitch | 0.082 rad |
| max torque | 11.69 Nm |
| saturation | 0.0 |
| final right force | 16.23 N |
| final right contacts | 4 |

## Fast Best Render

실행:

```bash
MUJOCO_GL=egl python3 scripts/render_force_ratio_sequence.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --poses configs/weight_shift_left065_contact_constrained_multipoint.json \
  --target-left-ratios 0.90 \
  --out-dir outputs/analysis/render_one_leg_support_fast_best_stl_footprint_contact_6s \
  --duration 6.0 --ramp 2.0 --hold 2.0 \
  --joint-kp 20 --joint-kd 12 --torque-limit 30 \
  --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 \
  --kforce 12 --force-sign 1 --force-role both --force-on all --force-side-mode opposite \
  --fps 15 --azimuth 135 --elevation -12 --distance 1.45
```

산출물:

- 영상:
  - `outputs/analysis/render_one_leg_support_fast_best_stl_footprint_contact_6s/force_ratio_sequence_render.mp4`
- timeline:
  - `outputs/analysis/render_one_leg_support_fast_best_stl_footprint_contact_6s/force_ratio_sequence_timeline.csv`

렌더 timeline 기준:

| 항목 | 값 |
|---|---:|
| gate time | 0.264 s |
| right force <= 5 N | 0.330 s |
| right contacts = 0 | 0.264 s |
| right contacts <= 1 | 1.716 s |
| roll <= 0.12 rad | 6.072 s |
| min right force | 0.0 N |
| final right force | 16.23 N |
| final right contacts | 4 |
| max roll | 0.100 rad |
| max torque | 11.69 Nm |

## Narrow Sweep

Fast sweep의 best 주변만 좁혀서 다시 확인했다.

실행:

```bash
python3 scripts/search_one_leg_support_gate.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --pose configs/weight_shift_left065_contact_constrained_multipoint.json \
  --out-dir outputs/analysis/one_leg_support_gate_narrow_stl_footprint_contact \
  --duration 6.0 --target-gate-time 0.5 \
  --right-force-gate 5.0 --right-contact-gate 0.0 --max-roll-gate 0.12 \
  --joint-kp 20 --joint-kd 12 --torque-limit 30 \
  --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 \
  --force-sign 1 --narrow
```

산출물:

- summary:
  - `outputs/analysis/one_leg_support_gate_narrow_stl_footprint_contact/one_leg_support_gate_summary.json`
- candidates:
  - `outputs/analysis/one_leg_support_gate_narrow_stl_footprint_contact/one_leg_support_gate_candidates.csv`

Best candidate:

| 항목 | 값 |
|---|---:|
| target left ratio | 0.94 |
| kforce | 10.0 |
| force role | both |
| force on | all |
| force side mode | opposite |
| internal gate time | 0.360 s |
| low force time | 0.478 s |
| no contact time | 0.360 s |
| max roll | 0.117 rad |
| max pitch | 0.071 rad |
| max torque | 11.82 Nm |
| saturation | 0.0 |
| final right force | 15.79 N |
| final right contacts | 11 |

## Narrow Best Render

실행:

```bash
MUJOCO_GL=egl python3 scripts/render_force_ratio_sequence.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --poses configs/weight_shift_left065_contact_constrained_multipoint.json \
  --target-left-ratios 0.94 \
  --out-dir outputs/analysis/render_one_leg_support_narrow_best_stl_footprint_contact_6s \
  --duration 6.0 --ramp 2.0 --hold 2.0 \
  --joint-kp 20 --joint-kd 12 --torque-limit 30 \
  --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 \
  --kforce 10 --force-sign 1 --force-role both --force-on all --force-side-mode opposite \
  --fps 15 --azimuth 135 --elevation -12 --distance 1.45
```

산출물:

- 영상:
  - `outputs/analysis/render_one_leg_support_narrow_best_stl_footprint_contact_6s/force_ratio_sequence_render.mp4`
- timeline:
  - `outputs/analysis/render_one_leg_support_narrow_best_stl_footprint_contact_6s/force_ratio_sequence_timeline.csv`

렌더 timeline 기준:

| 항목 | 값 |
|---|---:|
| gate time | 0.330 s |
| right force <= 5 N | 0.528 s |
| right contacts = 0 | 0.330 s |
| right contacts <= 1 | 2.244 s |
| roll <= 0.12 rad | 6.072 s |
| min right force | 0.0 N |
| final right force | 15.79 N |
| final right contacts | 11 |
| max roll | 0.117 rad |
| max pitch | 0.071 rad |
| max torque | 11.82 Nm |

## 판단

이번 단계는 이전 foot lift probe보다 의미 있는 개선을 보였다.

- 오른발 하중은 순간적으로 `0 N`까지 떨어진다.
- 오른발 무접촉 상태도 발생한다.
- roll은 `0.12 rad` 이내로 유지된다.
- torque saturation은 없다.

하지만 one-leg support gate는 아직 통과하지 못했다.

- 목표 gate: `0.5 s`
- 현재 best internal gate: `0.360 s`
- 현재 best render timeline gate: `0.330 s`

특히 narrow best에서는 `right force <= 5 N` 시간이 `0.528 s`로 목표를 넘었지만, `right contacts = 0` 시간이 `0.330 s`라서 최종 gate를 막고 있다. 즉 하중은 거의 왼발로 옮길 수 있지만, 오른발 접촉을 완전히 끊고 유지하는 능력이 아직 부족하다.

## 현재 Gate 상태

| 단계 | 결과 |
|---|---|
| standing | 통과 |
| weight shift | 통과 |
| right unload | 통과 |
| transient toe-off | 통과 |
| foot-frame IK pose generation | 통과 |
| one-leg support 0.5초 | 실패 |
| 2 mm clearance 0.5초 유지 | 실패 |

## 다음 조치

다음 단계는 오른발 접촉 해제를 조금 더 오래 유지하도록 `unload + small lift`를 결합하는 것이다.

권장 방향:

1. narrow best controller를 기준으로 사용한다.
2. 오른발 IK lift를 바로 크게 넣지 말고 `1~2 mm`만 추가한다.
3. 목표는 `right contacts = 0` 시간을 `0.5 s` 이상으로 늘리는 것이다.
4. 이때 roll gate는 이미 임계값 근처이므로 `max roll <= 0.12 rad`를 넘기지 않는지 우선 확인한다.
5. 이 조합도 실패하면 설계/제어 병목은 발목 roll 축, COM 위치, 발바닥 support polygon 쪽으로 더 강하게 의심한다.
