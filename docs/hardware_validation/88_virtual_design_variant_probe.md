# 88. 가상 설계 변형 Probe

## 목적

이 단계의 목적은 실제 CAD를 수정하기 전에 MuJoCo 모델에서 가상 설계 변형을 만들어, 오른발 하중 제거 실패 원인이 어디에 가까운지 분리하는 것이다.

이전 단계에서 확인한 핵심 문제는 다음이었다.

- 안정 `0.65` weight shift 자세에서도 COM 투영점이 왼발 단독 support polygon 밖에 있음
- 왼발 단독 기준 margin 약 `-30 mm`
- 오른발 unload `0.70+` 시도는 roll 붕괴로 실패
- torque saturation은 없었으므로 모터 토크 부족이 1차 원인으로 보이지 않음

따라서 이번 단계에서는 다음 가상 변경을 비교했다.

1. 발바닥 support polygon 확대
2. base COM 이동
3. 발바닥 확대 + base COM 이동 조합

## 사용 모델과 스크립트

- 기준 모델:
  - `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics_multipoint_fullheight_contact.xml`
- 가상 변형 생성:
  - `scripts/build_virtual_design_variant.py`
- COM support polygon 감사:
  - `scripts/audit_com_support_polygon.py`
- 동역학 transition/render:
  - `scripts/render_pose_transition.py`
- force ratio feedback probe:
  - `scripts/render_force_ratio_sequence.py`

## 생성한 가상 모델

### 1. 발바닥 폭 2.0배

- 모델:
  - `envs/robots/urdf_f_link/URDF_F_link_virtual_sole_y2p0.xml`
- 변경:
  - 좌우 foot sole pad의 local Y 방향 offset과 halfsize를 2.0배
- 보고서:
  - `outputs/analysis/virtual_design_variants/sole_y2p0_report.json`

### 2. base COM +80 mm 이동

- 모델:
  - `envs/robots/urdf_f_link/URDF_F_link_virtual_base_com_yplus80mm.xml`
- 변경:
  - `base_link` inertial pos의 local Y를 `+0.08 m` 이동
- 보고서:
  - `outputs/analysis/virtual_design_variants/base_com_yplus80mm_report.json`

### 3. 발바닥 폭 2.0배 + base COM +80 mm

- 모델:
  - `envs/robots/urdf_f_link/URDF_F_link_virtual_sole_y2p0_base_com_yplus80mm.xml`
- 보고서:
  - `outputs/analysis/virtual_design_variants/sole_y2p0_base_com_yplus80mm_report.json`

### 4. 발바닥 폭 2.5배

- 모델:
  - `envs/robots/urdf_f_link/URDF_F_link_virtual_sole_y2p5.xml`
- 변경:
  - 좌우 foot sole pad의 local Y 방향 offset과 halfsize를 2.5배
- 보고서:
  - `outputs/analysis/virtual_design_variants/sole_y2p5_report.json`

## 정적 Support Polygon 결과

기준 자세는 모두 `configs/weight_shift_left065_contact_constrained_multipoint.json`이다.

| 모델 | 왼발 단독 polygon margin | 판단 |
|---|---:|---|
| 기준 모델 | -30.0 mm | 왼발 단독 지지 불가 |
| sole Y 2.0배 | +5.1 mm | 왼발 단독 지지 조건에 진입 |
| base COM +80 mm | -4.6 mm | 크게 개선되지만 아직 외부 |
| sole Y 2.0배 + base COM +80 mm | +17.2 mm | 정적 조건은 충분히 개선 |
| sole Y 2.5배 | +17.6 mm | 정적 조건은 충분히 개선 |

산출물:

- `outputs/analysis/com_support_polygon_left065_virtual_sole_y2p0/com_support_polygon_audit.json`
- `outputs/analysis/com_support_polygon_left065_virtual_base_com_yplus80mm/com_support_polygon_audit.json`
- `outputs/analysis/com_support_polygon_left065_virtual_sole_y2p0_base_com_yplus80mm/com_support_polygon_audit.json`
- `outputs/analysis/com_support_polygon_left065_virtual_sole_y2p5/com_support_polygon_audit.json`

## 동역학 Transition 결과

중립 standing에서 안정 `0.65` 자세로 이동하는 transition을 각 모델에서 확인했다.

| 모델 | 최종 left ratio | 최종 roll | max torque | saturation | 판단 |
|---|---:|---:|---:|---:|---|
| sole Y 2.0배 | 0.588 | -0.032 rad | 8.41 Nm | 0.0 | 안정 |
| base COM +80 mm | 0.653 | 2.828 rad | 19.09 Nm | 0.0 | 실패 |
| sole Y 2.0배 + base COM +80 mm | 0.634 | 2.767 rad | 30.00 Nm | 0.0015 | 실패 |
| sole Y 2.5배 | 0.593 | -0.030 rad | 11.20 Nm | 0.0 | 안정 |

산출물:

- `outputs/analysis/render_left065_virtual_sole_y2p0_8s/pose_transition_render.mp4`
- `outputs/analysis/render_left065_virtual_base_com_yplus80mm_8s/pose_transition_render.mp4`
- `outputs/analysis/render_left065_virtual_sole_y2p0_base_com_yplus80mm_8s/pose_transition_render.mp4`
- `outputs/analysis/render_left065_virtual_sole_y2p5_8s/pose_transition_render.mp4`

## Force Ratio Feedback 결과

발바닥 확대 모델이 동역학적으로 안정했기 때문에, 같은 `0.65` 자세에서 force ratio feedback을 적용해 오른발 unload gate를 확인했다.

### 발바닥 폭 2.0배

| 목표 | 제어 조건 | 최종 left ratio | 최종 right force | max roll | max torque | saturation | 판단 |
|---:|---|---:|---:|---:|---:|---:|---|
| 0.70 | pitch/ankle, k=5 | 0.704 | 26.72 N | 0.076 rad | 15.33 Nm | 0.0 | 안정, force gate 약간 미달 |
| 0.70 | roll/ankle, k=5 | 0.703 | 26.81 N | 0.072 rad | 14.79 Nm | 0.0 | 안정, force gate 약간 미달 |
| 0.70 | both/all, k=2 | 0.712 | 26.02 N | 0.076 rad | 15.04 Nm | 0.0 | 안정, force gate 약간 미달 |
| 0.72 | both/all, k=2 | 0.714 | 25.87 N | 0.076 rad | 15.05 Nm | 0.0 | 안정, force gate 약간 미달 |

### 발바닥 폭 2.5배

| 목표 | 제어 조건 | 최종 left ratio | 최종 right force | max roll | max torque | saturation | 판단 |
|---:|---|---:|---:|---:|---:|---:|---|
| 0.72 | both/all, k=2 | 0.715 | 25.76 N | 0.049 rad | 20.06 Nm | 0.0 | 안정, force gate 약간 미달 |
| 0.75 | both/all, k=2 | 0.718 | 25.44 N | 0.049 rad | 20.08 Nm | 0.0 | 안정, force gate 약간 미달 |
| 0.75 | both/all, k=5 | 0.722 | 25.10 N | 0.049 rad | 19.80 Nm | 0.0 | 안정, 거의 통과 |
| 0.78 | both/all, k=5 | 0.729 | 24.52 N | 0.049 rad | 19.86 Nm | 0.0 | 통과 |

가장 좋은 결과:

- 모델:
  - `envs/robots/urdf_f_link/URDF_F_link_virtual_sole_y2p5.xml`
- 영상:
  - `outputs/analysis/force_ratio_virtual_sole_y2p5_hold065_target078_both_all_k5_pos/force_ratio_sequence_render.mp4`
- 최종 left ratio:
  - `0.729`
- 최종 right force:
  - `24.52 N`
- max roll:
  - `0.049 rad`
- max torque:
  - `19.86 Nm`
- torque saturation:
  - `0.0`

## 판단

이번 결과는 발바닥 support polygon 부족이 실제 병목 중 하나라는 강한 증거다.

기준 모델에서는 `0.70+` 오른발 unload가 roll 붕괴로 실패했다. 반면 발바닥 폭을 가상으로 2.5배 키운 모델에서는 다음 gate를 처음으로 통과했다.

- left force ratio `>= 0.70`
- right force `<= 25 N`
- max roll `<= 0.22 rad`
- final roll `<= 0.16 rad`
- torque saturation 없음

따라서 현재 하드웨어 설계에서 검토해야 할 방향은 다음이다.

1. 발바닥 폭 또는 실제 접촉면을 키우는 설계 변경
2. 접촉 pad가 실제 발바닥 전체를 대표하도록 contact 위치/크기 재검토
3. 발 자체를 키우지 않는다면, pelvis/torso/leg pose로 COM을 더 깊게 지지발 내부로 넣을 수 있는지 재검토

반면 base COM을 단순히 `+80 mm` 이동한 모델은 정적 margin은 개선했지만 동역학 transition에서 roll 붕괴가 발생했다. 따라서 base COM 이동은 방향성 자체가 틀렸다기보다, 단순 inertial 위치 변경만으로는 안정 해법이 아니며 너무 큰 변화였을 가능성이 높다.

## 설계 관점 결론

현재 설계는 양발 standing과 `0.65` weight shift까지 가능하지만, swing foot lift로 넘어가기 위한 한 발 지지 여유가 부족하다.

다만 발바닥 support polygon을 가상으로 키우면 오른발 unload gate를 통과한다. 따라서 “아무리 RL을 해도 절대 못 걷는다”는 결론은 아니다. 오히려 현재 실패는 다음 중 하나로 좁혀졌다.

- 발바닥 contact area가 실제/설계 의도보다 작게 모델링되었거나
- 실제 발 치수가 한 발 지지에 부족하거나
- 발 치수는 유지하되 COM 이동 가능한 자세/관절 범위/제어전략을 더 찾아야 하거나
- 위 세 가지가 함께 작용하고 있음

## 다음 조치

다음 단계에서는 `sole Y 2.5배`라는 비현실적 가상 변경을 그대로 설계안으로 쓰지 말고, 필요한 최소 발바닥 확장량을 찾는다.

1. sole Y scale sweep:
   - `1.0`, `1.25`, `1.5`, `1.75`, `2.0`, `2.25`, `2.5`
2. 각 scale에서 확인할 gate:
   - left force ratio `>= 0.70`
   - right force `<= 25 N`
   - max roll `<= 0.22 rad`
   - max torque `< 30 Nm`
   - saturation `0`
3. 최소 통과 scale을 찾은 뒤 실제 CAD foot width/contact pad 치수로 환산한다.

