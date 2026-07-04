# 89. 가상 발바닥 폭 Sweep

## 목적

이 단계의 목적은 이전 단계에서 확인한 “발바닥 support polygon 확대 시 오른발 unload gate 통과” 결과를 더 정량화하는 것이다.

이전 단계에서는 발바닥 Y 방향을 2.5배 키운 가상 모델이 오른발 unload gate를 통과했다. 하지만 2.5배는 실제 설계안으로 바로 쓰기에는 과한 값이다. 따라서 이번에는 필요한 최소 확장량을 찾기 위해 scale sweep을 수행했다.

## 사용 조건

- 기준 모델:
  - `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics_multipoint_fullheight_contact.xml`
- 기준 자세:
  - `configs/weight_shift_left065_contact_constrained_multipoint.json`
- sweep 스크립트:
  - `scripts/sweep_virtual_sole_width.py`
- 발바닥 scale:
  - `1.0`, `1.25`, `1.5`, `1.75`, `2.0`, `2.25`, `2.5`
- force feedback 조건:
  - target left ratio: `0.78`
  - `force_role = both`
  - `force_on = all`
  - `force_side_mode = same`
  - `kforce = 5`
- controller:
  - joint Kp `20`
  - joint Kd `12`
  - torque limit `30 Nm`
  - attitude Kp `0`
  - attitude Kd `1`
  - COM gain `1`

## Gate 기준

오른발 swing 준비 전 단계로 보기 위한 gate는 다음과 같이 두었다.

- final left force ratio `>= 0.70`
- final right force `<= 25 N`
- final roll `<= 0.16 rad`
- max roll `<= 0.22 rad`
- max torque `< 30 Nm`
- torque saturation `0`

## 산출물

- summary:
  - `outputs/analysis/virtual_sole_width_sweep_target078_k5/virtual_sole_width_sweep_summary.json`
- CSV:
  - `outputs/analysis/virtual_sole_width_sweep_target078_k5/virtual_sole_width_sweep.csv`
- 최소 통과 scale 영상:
  - `outputs/analysis/virtual_sole_width_sweep_target078_k5/force_ratio_y1p75_target0p78/force_ratio_sequence_render.mp4`
- 최소 통과 scale 모델:
  - `envs/robots/urdf_f_link/URDF_F_link_virtual_sweep_sole_y1p75.xml`

## 결과

| scale_y | 왼발 support margin | 최종 left ratio | 최종 right force | max roll | max torque | gate |
|---:|---:|---:|---:|---:|---:|---|
| 1.00 | -30.0 mm | 0.474 | 47.53 N | 3.141 rad | 19.13 Nm | 실패 |
| 1.25 | -21.2 mm | 0.497 | 45.48 N | 3.141 rad | 19.11 Nm | 실패 |
| 1.50 | -12.5 mm | 0.480 | 47.01 N | 3.141 rad | 21.35 Nm | 실패 |
| 1.75 | -3.7 mm | 0.728 | 24.62 N | 0.106 rad | 12.38 Nm | 통과 |
| 2.00 | +5.1 mm | 0.727 | 24.65 N | 0.076 rad | 14.87 Nm | 통과 |
| 2.25 | +13.8 mm | 0.728 | 24.61 N | 0.059 rad | 17.37 Nm | 통과 |
| 2.50 | +17.6 mm | 0.729 | 24.52 N | 0.049 rad | 19.86 Nm | 통과 |

## 핵심 관찰

최소 gate 통과 scale은 `1.75배`다.

흥미로운 점은 `scale_y = 1.75`에서 왼발 단독 support polygon margin이 아직 `-3.7 mm`라는 것이다. 즉 정적 COM 투영점은 왼발 polygon 바깥에 아주 조금 남아 있지만, 동역학 접촉/제어 조건에서는 오른발 unload gate를 통과했다.

반면 `scale_y = 1.50`까지는 모두 roll이 거의 `pi`까지 커지며 실패했다. 따라서 현재 모델에서는 발바닥 Y 방향 support가 어느 임계값을 넘기 전까지는 하중 이동이 안정화되지 않는다.

## 설계 환산

현재 multipoint pad의 local Y 기준 값은 다음과 같다.

- pad center Y:
  - `±17.5 mm`
- pad halfsize Y:
  - `19.5 mm`
- 전체 local Y 범위:
  - 약 `-37.0 mm` ~ `+37.0 mm`
- 총 폭:
  - 약 `74 mm`

`scale_y = 1.75`일 때:

- pad center Y:
  - `±30.625 mm`
- pad halfsize Y:
  - `34.125 mm`
- 전체 local Y 범위:
  - 약 `-64.75 mm` ~ `+64.75 mm`
- 총 폭:
  - 약 `129.5 mm`

따라서 현재 contact 모델 기준으로 오른발 unload gate를 통과하려면, 발바닥 Y 방향 유효 접촉 폭이 약 `74 mm`에서 최소 약 `130 mm` 수준으로 커져야 한다.

주의할 점:

- 이 값은 실제 CAD 발 폭 그 자체가 아니라 MuJoCo contact pad 기준의 유효 접촉 폭이다.
- 실제 발이 이미 더 넓은데 contact pad만 작게 들어간 것이라면, CAD 수정이 아니라 contact 모델 수정으로 해결될 수 있다.
- 실제 발의 유효 접촉면이 정말 74 mm 수준이라면, 한 발 지지 안정성을 위해 발 폭/접지면 확대를 설계 변경 후보로 봐야 한다.

## 판단

이번 sweep 결과는 발바닥 support polygon이 현재 오른발 unload 실패의 주요 병목이라는 판단을 강화한다.

특히 다음이 중요하다.

- `scale_y <= 1.50`: 실패
- `scale_y >= 1.75`: 성공
- torque saturation은 모든 통과 케이스에서 `0`
- 최소 성공 케이스의 max torque는 `12.38 Nm`로 여유가 있음

즉, 실패/성공을 가르는 요인은 토크 한계가 아니라 지지면 크기와 접촉 안정성이다.

## 다음 조치

다음 단계에서는 이 가상 contact 폭이 실제 CAD/URDF 발바닥과 맞는지 확인해야 한다.

1. 실제 STL/STEP에서 발바닥의 물리적 외곽 폭을 다시 확인한다.
2. 현재 MuJoCo contact pad 폭 `74 mm`가 실제 발 접촉면을 과소평가한 것인지 판단한다.
3. 실제 접촉 가능 폭이 `130 mm` 근처라면, contact model을 실제 치수에 맞게 수정한다.
4. 실제 발 폭 자체가 `130 mm`보다 작다면, foot width/sole extension을 하드웨어 설계 변경 후보로 올린다.
5. contact 모델을 실제 치수로 수정한 뒤, 다시 `standing -> 0.65 weight shift -> 0.78 unload -> swing foot lift` 순서로 gate를 검증한다.

