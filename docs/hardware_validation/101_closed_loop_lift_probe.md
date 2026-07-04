# 101. Closed-loop Lift Probe

## 목적

이 단계의 목적은 open-loop pose interpolation 대신, clearance와 roll을 보면서 lift 강도 `alpha`를 실시간으로 조절하는 간단한 closed-loop lift controller를 검증하는 것이다.

이전 단계 결과:

- `com_y005` 모델은 one-leg support gate를 조건부 통과했다.
- 그러나 open-loop lift에서는 `right clearance >= 2 mm` 유지 시간이 `0.198~0.264 s`에 머물렀다.

이번 controller:

- clearance가 목표보다 낮으면 lift `alpha` 증가
- roll이 커지면 lift `alpha` 감소
- return phase에서는 `alpha` 감소

## 사용 모델

- `envs/robots/urdf_f_link/URDF_F_link_virtual_com_gate_com_y005.xml`

기준 pose:

- `configs/weight_shift_left065_contact_constrained_multipoint.json`

Lift pose:

- `configs/right_foot_lift002_ik_stl_footprint_contact.json`

## 추가 스크립트

- `scripts/render_closed_loop_lift_sequence.py`

## 기본 Closed-loop

실행:

```bash
MUJOCO_GL=egl python3 scripts/render_closed_loop_lift_sequence.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_com_gate_com_y005.xml \
  --start-pose configs/weight_shift_left065_contact_constrained_multipoint.json \
  --lift-pose configs/right_foot_lift002_ik_stl_footprint_contact.json \
  --out-dir outputs/analysis/render_com_y005_closed_loop_lift002_8s \
  --duration 8.0 --lift-start 1.0 --lift-end 4.0 --return-end 6.0 \
  --target-clearance 0.002 --alpha-max 1.25 \
  --alpha-up-rate 1.4 --alpha-down-rate 2.2 \
  --roll-soft-limit 0.10 --roll-hard-limit 0.14 \
  --target-left-ratio 0.94 \
  --joint-kp 20 --joint-kd 12 --torque-limit 30 \
  --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 \
  --kforce 12 --force-sign 1 --force-role both --force-on all --force-side-mode opposite \
  --fps 15 --azimuth 135 --elevation -12 --distance 1.45
```

산출물:

- 영상:
  - `outputs/analysis/render_com_y005_closed_loop_lift002_8s/closed_loop_lift_render.mp4`
- timeline:
  - `outputs/analysis/render_com_y005_closed_loop_lift002_8s/closed_loop_lift_timeline.csv`

결과:

| 항목 | 값 |
|---|---:|
| combined lift gate | 0.198 s |
| clearance gate | 0.198 s |
| one-leg gate | 0.528 s |
| clearance > 0 | 0.528 s |
| clearance >= 2 mm | 0.198 s |
| clearance >= 5 mm | 0.132 s |
| right force <= 5 N | 0.726 s |
| right contacts = 0 | 0.528 s |
| max alpha | 1.25 |
| max clearance | 9.92 mm |
| max roll | 0.286 rad |
| max torque | 11.80 Nm |

## 보수형 Closed-loop

roll 억제를 더 강하게 하기 위해 `alpha-max`와 roll limit을 낮췄다.

실행:

```bash
MUJOCO_GL=egl python3 scripts/render_closed_loop_lift_sequence.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_com_gate_com_y005.xml \
  --start-pose configs/weight_shift_left065_contact_constrained_multipoint.json \
  --lift-pose configs/right_foot_lift002_ik_stl_footprint_contact.json \
  --out-dir outputs/analysis/render_com_y005_closed_loop_lift002_safe_8s \
  --duration 8.0 --lift-start 1.0 --lift-end 3.5 --return-end 5.5 \
  --target-clearance 0.002 --alpha-max 0.9 \
  --alpha-up-rate 0.9 --alpha-down-rate 3.0 \
  --roll-soft-limit 0.08 --roll-hard-limit 0.11 \
  --target-left-ratio 0.94 \
  --joint-kp 20 --joint-kd 12 --torque-limit 30 \
  --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 \
  --kforce 12 --force-sign 1 --force-role both --force-on all --force-side-mode opposite \
  --fps 15 --azimuth 135 --elevation -12 --distance 1.45
```

산출물:

- 영상:
  - `outputs/analysis/render_com_y005_closed_loop_lift002_safe_8s/closed_loop_lift_render.mp4`
- timeline:
  - `outputs/analysis/render_com_y005_closed_loop_lift002_safe_8s/closed_loop_lift_timeline.csv`

결과:

| 항목 | 값 |
|---|---:|
| combined lift gate | 0.198 s |
| clearance gate | 0.198 s |
| one-leg gate | 0.396 s |
| clearance > 0 | 0.396 s |
| clearance >= 2 mm | 0.198 s |
| clearance >= 5 mm | 0.132 s |
| right force <= 5 N | 0.660 s |
| right contacts = 0 | 0.396 s |
| max alpha | 0.90 |
| max clearance | 9.92 mm |
| max roll | 0.260 rad |
| max torque | 11.80 Nm |

## 비교

| 항목 | open-loop 2 mm | closed-loop | safe closed-loop |
|---|---:|---:|---:|
| combined lift gate | 0.198 s | 0.198 s | 0.198 s |
| one-leg gate | 0.462 s | 0.528 s | 0.396 s |
| clearance >= 2 mm | 0.198 s | 0.198 s | 0.198 s |
| right contacts = 0 | 0.462 s | 0.528 s | 0.396 s |
| max roll | 0.322 rad | 0.286 rad | 0.260 rad |
| max torque | 11.80 Nm | 11.80 Nm | 11.80 Nm |

## 판단

Closed-loop alpha 조절은 일부 지표를 개선했다.

- one-leg gate가 `0.462 s -> 0.528 s`로 증가
- right contacts = 0 시간도 `0.528 s`까지 증가
- max roll은 open-loop보다 조금 감소

하지만 핵심 목표인 `right clearance >= 2 mm` 유지 시간은 `0.198 s`에서 개선되지 않았다.

즉 현재 병목은 단순한 lift alpha schedule 문제가 아니다. 같은 lift pose family 안에서는 발이 순간적으로는 뜨지만, 2 mm 이상 clearance를 오래 유지할 수 없다.

## 현재 Gate 상태

| 단계 | 결과 |
|---|---|
| standing | 통과 |
| weight shift | 통과 |
| right unload | 통과 |
| transient toe-off | 통과 |
| virtual COM sensitivity | one-leg support 조건부 통과 |
| closed-loop lift alpha | one-leg gate 개선, clearance gate 실패 |
| 2 mm clearance 0.5초 유지 | 실패 |

## 다음 조치

다음 단계부터는 단순 alpha feedback이 아니라 swing foot의 Cartesian target을 직접 제어해야 한다.

권장 방향:

1. 오른발 sole corner Z를 매 timestep 측정한다.
2. `right_hip_pitch`, `right_knee_pitch`, `right_ankle_pitch`, `right_ankle_roll`에 clearance error 기반 보정 torque 또는 target offset을 직접 넣는다.
3. roll이 커지면 swing foot target이 아니라 support leg roll target을 보정한다.
4. 이 구조는 사실상 간단한 task-space controller이므로, 이후 RL 초기 policy 또는 reward 설계의 기준이 된다.
5. 하드웨어 측면에서는 `base COM +Y 약 5 mm` 조정 가능성을 유지하고, 발 toe/heel 접촉 길이 보강 후보를 계속 보류한다.
