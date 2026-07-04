# 99. Virtual COM Sensitivity

## 목적

이 단계의 목적은 `base_link` inertial COM을 가상으로 이동시켜 one-leg support gate가 개선되는지 확인하는 것이다.

이전 단계에서 발바닥 support polygon 민감도는 확인됐다.

- 발바닥 local X 방향 `1.25x` 확장 시 gate time이 `0.360 s -> 0.468 s`로 개선
- 하지만 목표 `0.5 s`에는 미달

이번 단계에서는 상체/베이스 질량중심 배치가 더 큰 병목인지 확인했다.

## 좌표 방향 확인

COM/support polygon audit 기준:

| 항목 | MuJoCo X | MuJoCo Y |
|---|---:|---:|
| COM projection | 0.0519 m | -0.0434 m |
| left foot support centroid | 0.0616 m | 0.0299 m |
| left support y_min | - | -0.0361 m |

현재 COM은 왼발 support polygon보다 MuJoCo `Y` 음수 방향으로 약 `7~8 mm` 바깥에 있다.

따라서 왼발 지지 쪽으로 COM을 이동시키는 방향은 `base_link inertial +Y`로 설정했다.

## 사용 모델

원본:

- `envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml`

기준 pose:

- `configs/weight_shift_left065_contact_constrained_multipoint.json`

## 추가 스크립트

- `scripts/sweep_virtual_com_design_gate.py`

역할:

1. `scripts/build_virtual_design_variant.py`로 `base_link` inertial COM 이동 모델 생성
2. `scripts/audit_com_support_polygon.py`로 COM/support polygon 재계산
3. `scripts/search_one_leg_support_gate.py --narrow`로 one-leg support gate 평가
4. CSV와 summary JSON 저장

## COM 단독 Sweep

실행:

```bash
python3 scripts/sweep_virtual_com_design_gate.py \
  --source-model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --pose configs/weight_shift_left065_contact_constrained_multipoint.json \
  --out-dir outputs/analysis/virtual_com_design_gate_sweep_stl_footprint_contact \
  --duration 6.0 --target-gate-time 0.5 --variants focused
```

산출물:

- summary:
  - `outputs/analysis/virtual_com_design_gate_sweep_stl_footprint_contact/virtual_com_design_gate_sweep_summary.json`
- CSV:
  - `outputs/analysis/virtual_com_design_gate_sweep_stl_footprint_contact/virtual_com_design_gate_sweep.csv`

결과:

| Variant | base COM shift Y | COM Y | left margin | gate time | no contact time | max roll | max torque | 판정 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| baseline | 0 mm | -43.37 mm | -8.19 mm | 0.360 s | 0.360 s | 0.117 rad | 11.82 Nm | 실패 |
| com_y005 | +5 mm | -41.79 mm | -6.61 mm | 0.512 s | 0.512 s | 0.177 rad | 11.80 Nm | 통과 |
| com_y010 | +10 mm | -40.20 mm | -5.02 mm | 0.358 s | 0.358 s | 0.186 rad | 14.48 Nm | 실패 |
| com_y020 | +20 mm | -37.03 mm | -1.86 mm | 0.328 s | 0.328 s | 0.201 rad | 11.81 Nm | 실패 |
| com_y040 | +40 mm | -30.69 mm | +4.48 mm | 0.330 s | 0.506 s | 3.141 rad | 22.41 Nm | 실패 |
| com_y060 | +60 mm | -24.34 mm | +10.82 mm | 0.258 s | 0.470 s | 3.141 rad | 23.47 Nm | 실패 |

Best:

- `com_y005`
- `base_link` inertial COM local Y를 `+5 mm` 이동
- internal timestep 기준 gate time `0.512 s`
- 최초로 목표 `0.5 s`를 통과

주의:

- `com_y005`의 gate time은 통과했지만, 전체 6초 시뮬레이션 중 max roll은 `0.177 rad`까지 증가했다.
- gate 계산은 `right force <= 5 N`, `right contacts = 0`, `abs(roll) <= 0.12 rad`가 동시에 만족되는 시간만 합산한다.
- 따라서 “순간 gate 통과”이지, 전체 6초가 완전히 안정적이라는 뜻은 아니다.

## COM +5 mm Render

실행:

```bash
MUJOCO_GL=egl python3 scripts/render_force_ratio_sequence.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_com_gate_com_y005.xml \
  --poses configs/weight_shift_left065_contact_constrained_multipoint.json \
  --target-left-ratios 0.94 \
  --out-dir outputs/analysis/render_virtual_com_y005_best_6s \
  --duration 6.0 --ramp 2.0 --hold 2.0 \
  --joint-kp 20 --joint-kd 12 --torque-limit 30 \
  --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 \
  --kforce 12 --force-sign 1 --force-role both --force-on all --force-side-mode opposite \
  --fps 15 --azimuth 135 --elevation -12 --distance 1.45
```

산출물:

- 영상:
  - `outputs/analysis/render_virtual_com_y005_best_6s/force_ratio_sequence_render.mp4`
- timeline:
  - `outputs/analysis/render_virtual_com_y005_best_6s/force_ratio_sequence_timeline.csv`

렌더 timeline 기준 비교:

| 항목 | baseline | long_x125 | com_y005 |
|---|---:|---:|---:|
| gate time | 0.330 s | 0.462 s | 0.462 s |
| right force <= 5 N | 0.528 s | 0.594 s | 0.660 s |
| right contacts = 0 | 0.330 s | 0.462 s | 0.462 s |
| right contacts <= 1 | 2.244 s | 3.300 s | 2.046 s |
| max roll | 0.117 rad | 0.131 rad | 0.177 rad |
| max pitch | 0.071 rad | 0.085 rad | 0.068 rad |
| max torque | 11.82 Nm | 12.92 Nm | 11.80 Nm |
| final right force | 15.79 N | 14.97 N | 16.26 N |
| final right contacts | 11 | 4 | 12 |

렌더 timeline은 15 fps 샘플링이라 내부 timestep보다 보수적으로 `0.462 s`로 계산된다. 내부 gate 통과 여부는 `search_one_leg_support_gate.py`의 timestep 기준 `0.512 s`를 기준으로 본다.

## COM + Support Polygon 결합 Sweep

실행:

```bash
python3 scripts/sweep_virtual_com_design_gate.py \
  --source-model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --pose configs/weight_shift_left065_contact_constrained_multipoint.json \
  --out-dir outputs/analysis/virtual_com_support_combined_gate_sweep_stl_footprint_contact \
  --duration 6.0 --target-gate-time 0.5 --variants combined
```

산출물:

- summary:
  - `outputs/analysis/virtual_com_support_combined_gate_sweep_stl_footprint_contact/virtual_com_design_gate_sweep_summary.json`
- CSV:
  - `outputs/analysis/virtual_com_support_combined_gate_sweep_stl_footprint_contact/virtual_com_design_gate_sweep.csv`

주요 결과:

| Variant | scale_x | COM shift Y | gate time | max roll | 판정 |
|---|---:|---:|---:|---:|---|
| baseline | 1.00 | 0 mm | 0.360 s | 0.117 rad | 실패 |
| com_y010 | 1.00 | +10 mm | 0.358 s | 0.186 rad | 실패 |
| com_y020 | 1.00 | +20 mm | 0.328 s | 0.201 rad | 실패 |
| com_y040 | 1.00 | +40 mm | 0.330 s | 3.141 rad | 실패 |
| long_x125 | 1.25 | 0 mm | 0.468 s | 0.131 rad | 실패 |
| long_x125_com_y010 | 1.25 | +10 mm | 0.300 s | 0.152 rad | 실패 |
| long_x125_com_y020 | 1.25 | +20 mm | 0.290 s | 0.179 rad | 실패 |
| long_x125_com_y040 | 1.25 | +40 mm | 0.292 s | 3.141 rad | 실패 |

추가 확인:

- `long_x125 + com_y005`도 별도로 생성해 확인했다.
- 결과는 gate time `0.290 s`, max roll `0.114 rad`로 오히려 나빠졌다.

즉 COM 이동과 발 길이 확장은 단순히 더하면 좋아지는 관계가 아니다.

## 판단

이번 단계에서 COM 민감도는 명확히 확인됐다.

확인된 사실:

- `base_link` inertial COM을 왼발 지지 방향으로 아주 작게, 약 `+5 mm` 이동하면 one-leg support gate가 내부 timestep 기준 `0.512 s`로 통과된다.
- 그러나 `+10 mm` 이상 이동하면 gate가 다시 나빠진다.
- `+40~60 mm`처럼 크게 이동하면 정적 support margin은 좋아지지만 roll collapse가 발생한다.
- 발 길이 `x=1.25`와 COM 이동을 결합해도 자동으로 좋아지지 않는다.

설계적으로 중요한 해석:

- 현재 모델은 상체/base 질량중심 위치에 매우 민감하다.
- 필요한 COM 조정량은 크지 않다. `base_link inertial +Y 5 mm` 수준에서 gate 통과가 발생했다.
- 너무 큰 COM 이동은 오히려 동역학적으로 불안정하다.
- 실제 CAD에서는 배터리, 제어보드, 상체 프레임, 배선, 외장 등 base_link에 포함될 질량 배치가 보행 가능성에 직접 영향을 줄 가능성이 크다.

## 현재 Gate 상태

| 단계 | 결과 |
|---|---|
| standing | 통과 |
| weight shift | 통과 |
| right unload | 통과 |
| transient toe-off | 통과 |
| foot-frame IK pose generation | 통과 |
| virtual support polygon sensitivity | 개선 확인, gate 미통과 |
| virtual COM sensitivity | 내부 timestep 기준 gate 통과 |
| one-leg support 0.5초 | 조건부 통과 |

## 설계 수정 후보

현재까지의 가상 설계 결과를 기준으로 우선순위는 다음과 같다.

1. 상체/base 질량중심을 왼발 지지 방향으로 약간 이동할 수 있는 배치 여유 확보
2. 발바닥 local X 방향 접촉 길이 또는 toe/heel 접촉점 위치 보강
3. 큰 폭 확장보다 접촉 길이와 COM 미세 조정 우선
4. COM을 크게 이동시키는 설계는 피한다

## 다음 조치

다음 단계는 `com_y005` 모델에서 실제 오른발 lift trajectory를 다시 붙이는 것이다.

Gate:

- one-leg support:
  - internal timestep 기준 이미 통과
- 다음 확인:
  - `right clearance >= 2 mm` for `>= 0.5 s`
  - max roll이 장기적으로 과도하게 증가하지 않는지 확인

권장 실행:

1. `URDF_F_link_virtual_com_gate_com_y005.xml` 모델 사용
2. 기존 `right_foot_lift002_ik_stl_footprint_contact.json` 또는 새 IK pose를 이 모델 기준으로 재생성
3. unload + lift sequence를 렌더링
4. clearance gate가 통과되면 RL 전 단계의 하드웨어 가능성은 상당히 긍정적으로 판단한다
