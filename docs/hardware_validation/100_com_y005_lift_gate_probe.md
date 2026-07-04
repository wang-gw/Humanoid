# 100. COM Y005 Lift Gate Probe

## 목적

이 단계의 목적은 `base_link` inertial COM을 `+Y 5 mm` 이동한 가상 모델에서 오른발 lift gate가 통과되는지 확인하는 것이다.

이전 단계 결과:

- `com_y005` 모델은 one-leg support gate를 internal timestep 기준 `0.512 s`로 통과했다.
- 하지만 이 결과는 오른발을 실제로 `2 mm` 이상 들어 올리는 swing clearance까지 포함한 것은 아니다.

이번 단계의 gate:

- `right clearance >= 2 mm`
- `right force <= 5 N`
- `right contacts = 0`
- `abs(roll) <= 0.12 rad`
- 위 조건을 동시에 `>= 0.5 s` 유지

## 사용 모델

- `envs/robots/urdf_f_link/URDF_F_link_virtual_com_gate_com_y005.xml`

기준 pose:

- `configs/weight_shift_left065_contact_constrained_multipoint.json`

Lift pose:

- `configs/right_foot_lift002_ik_stl_footprint_contact.json`
- `configs/right_foot_lift005_ik_stl_footprint_contact.json`

주의:

- `com_y005`는 inertial COM만 바뀐 모델이므로 kinematics는 원본과 동일하다.
- 따라서 기존 foot-frame IK pose를 그대로 사용할 수 있다.

## 2 mm IK Lift

실행:

```bash
MUJOCO_GL=egl python3 scripts/render_unload_lift_sequence.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_com_gate_com_y005.xml \
  --poses configs/weight_shift_left065_contact_constrained_multipoint.json configs/right_foot_lift002_ik_stl_footprint_contact.json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --out-dir outputs/analysis/render_com_y005_unload_lift002_ik_8s \
  --duration 8.0 --ramp 2.0 --hold 2.0 \
  --target-left-ratio 0.94 \
  --joint-kp 20 --joint-kd 12 --torque-limit 30 \
  --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 \
  --kforce 12 --force-sign 1 --force-role both --force-on all --force-side-mode opposite \
  --fps 15 --azimuth 135 --elevation -12 --distance 1.45
```

산출물:

- 영상:
  - `outputs/analysis/render_com_y005_unload_lift002_ik_8s/unload_lift_render.mp4`
- timeline:
  - `outputs/analysis/render_com_y005_unload_lift002_ik_8s/unload_lift_timeline.csv`

결과:

| 항목 | 값 |
|---|---:|
| one-leg gate | 0.462 s |
| clearance >= 2 mm, roll <= 0.12 | 0.198 s |
| combined lift gate | 0.198 s |
| clearance > 0 | 0.462 s |
| clearance >= 2 mm | 0.198 s |
| clearance >= 5 mm | 0.132 s |
| right force <= 5 N | 0.660 s |
| right contacts = 0 | 0.462 s |
| max clearance | 9.92 mm |
| final clearance | -0.403 mm |
| max roll | 0.322 rad |
| max pitch | 0.093 rad |
| max torque | 11.80 Nm |
| final right force | 20.35 N |
| final right contacts | 1 |

2 mm lift는 순간 clearance는 나오지만, `>=2 mm` 유지 시간이 `0.198 s`에 그쳤다. 또한 장기 roll이 `0.322 rad`까지 증가한다.

## 5 mm IK Lift

실행:

```bash
MUJOCO_GL=egl python3 scripts/render_unload_lift_sequence.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_com_gate_com_y005.xml \
  --poses configs/weight_shift_left065_contact_constrained_multipoint.json configs/right_foot_lift005_ik_stl_footprint_contact.json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --out-dir outputs/analysis/render_com_y005_unload_lift005_ik_8s \
  --duration 8.0 --ramp 2.0 --hold 2.0 \
  --target-left-ratio 0.94 \
  --joint-kp 20 --joint-kd 12 --torque-limit 30 \
  --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 \
  --kforce 12 --force-sign 1 --force-role both --force-on all --force-side-mode opposite \
  --fps 15 --azimuth 135 --elevation -12 --distance 1.45
```

산출물:

- 영상:
  - `outputs/analysis/render_com_y005_unload_lift005_ik_8s/unload_lift_render.mp4`
- timeline:
  - `outputs/analysis/render_com_y005_unload_lift005_ik_8s/unload_lift_timeline.csv`

결과:

| 항목 | 값 |
|---|---:|
| one-leg gate | 0.462 s |
| clearance >= 2 mm, roll <= 0.12 | 0.264 s |
| combined lift gate | 0.264 s |
| clearance > 0 | 0.528 s |
| clearance >= 2 mm | 0.264 s |
| clearance >= 5 mm | 0.132 s |
| right force <= 5 N | 0.660 s |
| right contacts = 0 | 0.528 s |
| max clearance | 9.92 mm |
| final clearance | -1.069 mm |
| max roll | 3.079 rad |
| max pitch | 0.351 rad |
| max torque | 23.63 Nm |
| final right force | 33.60 N |
| final right contacts | 1 |

5 mm lift는 `right contacts = 0` 시간이 `0.528 s`까지 늘지만, roll이 붕괴한다. 따라서 유효한 gate 통과로 볼 수 없다.

## Controller Sweep

2 mm/5 mm lift에 대해 target left ratio와 force gain을 작은 범위에서 sweep했다.

추가 스크립트:

- `scripts/search_unload_lift_gate.py`

실행:

```bash
python3 scripts/search_unload_lift_gate.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_com_gate_com_y005.xml \
  --start-pose configs/weight_shift_left065_contact_constrained_multipoint.json \
  --lift-poses configs/right_foot_lift002_ik_stl_footprint_contact.json configs/right_foot_lift005_ik_stl_footprint_contact.json \
  --out-dir outputs/analysis/com_y005_unload_lift_gate_search \
  --duration 8.0 --lift-ramp 2.0 --lift-hold 2.0 --return-ramp 2.0 \
  --target-gate-time 0.5 --target-clearance 0.002 \
  --right-force-gate 5.0 --right-contact-gate 0.0 --max-roll-gate 0.12 \
  --joint-kp 20 --joint-kd 12 --torque-limit 30 \
  --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 --force-sign 1
```

산출물:

- summary:
  - `outputs/analysis/com_y005_unload_lift_gate_search/unload_lift_gate_search_summary.json`
- candidates:
  - `outputs/analysis/com_y005_unload_lift_gate_search/unload_lift_gate_candidates.csv`

Best internal candidate:

| 항목 | 값 |
|---|---:|
| lift pose | 2 mm IK |
| target left ratio | 0.90 |
| kforce | 8.0 |
| one-leg gate | 0.168 s |
| clearance gate | 0.146 s |
| combined gate | 0.146 s |
| clearance >= 2 mm | 0.146 s |
| clearance >= 5 mm | 0.110 s |
| max clearance | 9.66 mm |
| max roll | 0.106 rad |
| max pitch | 0.083 rad |
| max torque | 11.68 Nm |
| saturation | 0.0 |

이 후보는 roll은 안정적이지만, clearance/one-leg gate 시간이 더 짧다.

## Sweep Best Render

실행:

```bash
MUJOCO_GL=egl python3 scripts/render_unload_lift_sequence.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_com_gate_com_y005.xml \
  --poses configs/weight_shift_left065_contact_constrained_multipoint.json configs/right_foot_lift002_ik_stl_footprint_contact.json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --out-dir outputs/analysis/render_com_y005_unload_lift_gate_search_best_8s \
  --duration 8.0 --ramp 2.0 --hold 2.0 \
  --target-left-ratio 0.90 \
  --joint-kp 20 --joint-kd 12 --torque-limit 30 \
  --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 \
  --kforce 8 --force-sign 1 --force-role both --force-on all --force-side-mode opposite \
  --fps 15 --azimuth 135 --elevation -12 --distance 1.45
```

산출물:

- 영상:
  - `outputs/analysis/render_com_y005_unload_lift_gate_search_best_8s/unload_lift_render.mp4`
- timeline:
  - `outputs/analysis/render_com_y005_unload_lift_gate_search_best_8s/unload_lift_timeline.csv`

렌더 결과:

| 항목 | 값 |
|---|---:|
| one-leg gate | 0.198 s |
| clearance >= 2 mm, roll <= 0.12 | 0.132 s |
| combined lift gate | 0.132 s |
| max clearance | 9.94 mm |
| max roll | 0.131 rad |
| max pitch | 0.111 rad |
| max torque | 11.79 Nm |
| final right force | 17.88 N |
| final right contacts | 4 |

## 비교

| 항목 | com_y005 one-leg only | com_y005 + 2 mm IK | com_y005 + 5 mm IK | sweep best |
|---|---:|---:|---:|---:|
| one-leg gate | 0.512 s internal / 0.462 s render | 0.462 s render | 0.462 s render | 0.146 s internal / 0.132 s render |
| clearance >= 2 mm gate | - | 0.198 s | 0.264 s | 0.146 s internal / 0.132 s render |
| right contacts = 0 | 0.512 s internal / 0.462 s render | 0.462 s | 0.528 s | 0.168 s internal / 0.198 s render |
| max roll | 0.177 rad | 0.322 rad | 3.079 rad | 0.106 rad internal / 0.131 rad render |
| max torque | 11.80 Nm | 11.80 Nm | 23.63 Nm | 11.68 Nm |

## 판단

`com_y005`는 one-leg support gate를 개선했지만, 현재 open-loop lift trajectory까지 해결하지는 못했다.

확인된 사실:

- COM +5 mm 이동은 오른발 하중 해제와 무접촉 시간을 개선한다.
- 하지만 발을 실제로 `2 mm` 이상 들어 올리는 순간 roll 안정성이 나빠지거나 clearance 지속시간이 짧아진다.
- 5 mm lift는 접촉 해제 시간은 늘리지만 roll collapse가 발생한다.
- controller gain/target ratio를 낮추면 roll은 안정되지만 clearance gate가 더 짧아진다.

따라서 현재 병목은 더 이상 단순한 mass placement만은 아니다. `one-leg support`와 `swing foot clearance`를 동시에 만족하려면 lift trajectory 자체가 더 지능적이어야 한다.

## 현재 Gate 상태

| 단계 | 결과 |
|---|---|
| standing | 통과 |
| weight shift | 통과 |
| right unload | 통과 |
| transient toe-off | 통과 |
| foot-frame IK pose generation | 통과 |
| virtual COM sensitivity | one-leg support 조건부 통과 |
| com_y005 lift gate | 실패 |
| 2 mm clearance 0.5초 유지 | 실패 |

## 다음 조치

다음 단계는 단순한 pose interpolation이 아니라 closed-loop foot clearance controller 또는 RL 준비 단계로 넘어가야 한다.

권장 방향:

1. swing foot height를 feedback으로 제어한다.
2. roll이 커지면 lift target을 줄이거나 return을 빠르게 하는 safety controller를 둔다.
3. reward/gate는 `right clearance`, `base roll`, `contact force`, `torque`를 동시에 보게 한다.
4. 하드웨어 설계 측면에서는 `base COM +Y 약 5 mm` 조정 가능성과 toe/heel 접촉 길이 보강을 CAD 수정 후보로 기록한다.
5. 다음 검증은 RL 또는 MPC 성격의 closed-loop stepping controller에서 수행한다.
