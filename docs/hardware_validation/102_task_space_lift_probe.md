# 102. Task-space Lift Probe

## 목적

이 단계의 목적은 단순 `alpha` 조절이 아니라, 오른발 sole clearance error를 직접 보고 오른쪽 다리 관절 target offset을 보정하는 task-space 성격의 lift controller를 검증하는 것이다.

이전 단계 결과:

- closed-loop alpha controller는 one-leg gate를 개선했다.
- 하지만 `right clearance >= 2 mm` 유지 시간은 `0.198 s`에서 개선되지 않았다.

이번 controller:

- 매 timestep 현재 floating-base 상태에서 오른발 sole clearance를 측정
- 오른발 관절을 미소 perturb하여 clearance Jacobian을 finite difference로 근사
- clearance error를 줄이는 방향으로 target offset을 누적
- roll이 커지면 offset을 감소

## 사용 모델

- `envs/robots/urdf_f_link/URDF_F_link_virtual_com_gate_com_y005.xml`

기준 pose:

- `configs/weight_shift_left065_contact_constrained_multipoint.json`

## 추가 스크립트

- `scripts/render_task_space_lift_sequence.py`

Task-space 보정 관절:

- `right_hip_roll`
- `right_hip_pitch`
- `right_knee_pitch`
- `right_ankle_pitch`
- `right_ankle_roll`

## 기본 Task-space Controller

실행:

```bash
MUJOCO_GL=egl python3 scripts/render_task_space_lift_sequence.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_com_gate_com_y005.xml \
  --start-pose configs/weight_shift_left065_contact_constrained_multipoint.json \
  --out-dir outputs/analysis/render_com_y005_task_space_lift_8s \
  --duration 8.0 --lift-start 1.0 --lift-end 4.0 --return-end 6.0 \
  --target-clearance 0.002 \
  --task-gain 0.9 --task-decay 1.6 --jac-step 1e-4 --damping 1e-4 --max-dq-step 0.012 \
  --roll-soft-limit 0.10 --roll-hard-limit 0.14 \
  --target-left-ratio 0.94 \
  --joint-kp 20 --joint-kd 12 --torque-limit 30 \
  --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 \
  --kforce 12 --force-sign 1 --force-role both --force-on all --force-side-mode opposite \
  --fps 15 --azimuth 135 --elevation -12 --distance 1.45
```

산출물:

- 영상:
  - `outputs/analysis/render_com_y005_task_space_lift_8s/task_space_lift_render.mp4`
- timeline:
  - `outputs/analysis/render_com_y005_task_space_lift_8s/task_space_lift_timeline.csv`

결과:

| 항목 | 값 |
|---|---:|
| combined lift gate | 0.264 s |
| clearance gate | 0.264 s |
| one-leg gate | 0.264 s |
| clearance > 0 | 1.320 s |
| clearance >= 2 mm | 0.990 s |
| clearance >= 5 mm | 0.792 s |
| right contacts = 0 | 1.320 s |
| right force <= 5 N | 1.782 s |
| max offset norm | 0.436 rad |
| max clearance | 92.51 mm |
| max roll | 3.128 rad |
| max pitch | 1.292 rad |
| max torque | 23.34 Nm |

기본 controller는 clearance를 크게 만들 수 있었지만 roll collapse가 발생했다. 따라서 유효한 gate 통과로 볼 수 없다.

## 안전형 Task-space Controller

관절 offset과 roll limit을 더 보수적으로 제한했다.

실행:

```bash
MUJOCO_GL=egl python3 scripts/render_task_space_lift_sequence.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_com_gate_com_y005.xml \
  --start-pose configs/weight_shift_left065_contact_constrained_multipoint.json \
  --out-dir outputs/analysis/render_com_y005_task_space_lift_safe_8s \
  --duration 8.0 --lift-start 1.0 --lift-end 3.5 --return-end 5.5 \
  --target-clearance 0.002 \
  --task-gain 0.25 --task-decay 3.0 --jac-step 1e-4 --damping 5e-4 --max-dq-step 0.003 \
  --roll-soft-limit 0.07 --roll-hard-limit 0.10 \
  --target-left-ratio 0.94 \
  --joint-kp 20 --joint-kd 12 --torque-limit 30 \
  --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 \
  --kforce 12 --force-sign 1 --force-role both --force-on all --force-side-mode opposite \
  --fps 15 --azimuth 135 --elevation -12 --distance 1.45
```

산출물:

- 영상:
  - `outputs/analysis/render_com_y005_task_space_lift_safe_8s/task_space_lift_render.mp4`
- timeline:
  - `outputs/analysis/render_com_y005_task_space_lift_safe_8s/task_space_lift_timeline.csv`

결과:

| 항목 | 값 |
|---|---:|
| combined lift gate | 0.396 s |
| clearance gate | 0.396 s |
| one-leg gate | 0.396 s |
| clearance > 0 | 0.396 s |
| clearance >= 2 mm | 0.396 s |
| clearance >= 5 mm | 0.264 s |
| right contacts = 0 | 0.396 s |
| right force <= 5 N | 0.528 s |
| max offset norm | 0.295 rad |
| max clearance | 9.92 mm |
| final clearance | -0.126 mm |
| max roll | 0.179 rad |
| max pitch | 0.116 rad |
| max torque | 11.80 Nm |
| final right force | 16.84 N |
| final right contacts | 6 |

## 비교

| 항목 | open-loop 2 mm | alpha closed-loop | task-space safe |
|---|---:|---:|---:|
| combined lift gate | 0.198 s | 0.198 s | 0.396 s |
| clearance >= 2 mm | 0.198 s | 0.198 s | 0.396 s |
| right contacts = 0 | 0.462 s | 0.528 s | 0.396 s |
| max roll | 0.322 rad | 0.286 rad | 0.179 rad |
| max torque | 11.80 Nm | 11.80 Nm | 11.80 Nm |

## 판단

Task-space 방식은 처음으로 clearance gate를 의미 있게 개선했다.

확인된 사실:

- 단순 alpha 제어는 clearance gate를 늘리지 못했다.
- task-space 보정은 `combined lift gate`를 `0.198 s -> 0.396 s`까지 늘렸다.
- torque saturation 없이 개선됐다.
- 하지만 목표 `0.5 s`에는 아직 미달이다.
- 공격형 설정은 충분한 clearance를 만들 수 있지만 roll collapse가 발생한다.

따라서 현재 모델은 “발을 띄울 수는 있지만, 안정적으로 오래 유지할 closed-loop balance가 부족하다”는 쪽으로 해석된다. 그래도 task-space 방식에서 gate가 크게 개선됐기 때문에, RL/MPC 계열 controller를 붙이면 보행 가능성이 완전히 닫혀 있다고 보기는 어렵다.

## 현재 Gate 상태

| 단계 | 결과 |
|---|---|
| standing | 통과 |
| weight shift | 통과 |
| right unload | 통과 |
| virtual COM sensitivity | one-leg support 조건부 통과 |
| closed-loop alpha | clearance gate 개선 없음 |
| task-space lift | 크게 개선, 0.5초 미달 |
| 2 mm clearance 0.5초 유지 | 실패 |

## 다음 조치

다음 단계는 설계 검증 결론을 정리하고 RL 준비 여부를 판단하는 것이다.

권장 판단:

1. 현재 하드웨어 형상은 완전히 불가능하다고 볼 수준은 아니다.
2. 다만 open-loop나 단순 PD trajectory만으로 보행 가능하다고 판단하면 안 된다.
3. 실제 RL에서는 reward에 `clearance`, `roll`, `right/left contact force`, `torque`, `COM`을 동시에 넣어야 한다.
4. CAD 설계 후보로는 `base COM +Y 약 5 mm`, toe/heel 접촉 길이 보강을 유지한다.
5. 다음 문서는 지금까지의 gate 결과를 종합한 “RL 진행 가능성 및 하드웨어 수정 후보”로 정리한다.
