# 86. 오른발 하중 제거 Probe

## 목적

이 단계의 목적은 이전 단계에서 확보한 안정 자세인 `left force ratio ~= 0.65`에서 오른발 하중을 더 줄여, 이후 오른발 swing foot lift 테스트로 넘어갈 수 있는지 확인하는 것이다.

이번 테스트는 보행 RL로 바로 넘어가기 전의 하드웨어 가능성 검증이다. 즉, 현재 형상/질량/접촉 모델이 한 발 지지에 가까운 상태를 물리적으로 유지할 수 있는지 확인한다.

## 시작점

- 기준 모델:
  - `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics_multipoint_fullheight_contact.xml`
- 시작 자세:
  - `configs/weight_shift_left065_contact_constrained_multipoint.json`
- 이전 안정 결과:
  - 12초 검증에서 최종 left force ratio `0.654`
  - 최종 roll `-0.023 rad`
  - 최종 pitch `-0.093 rad`
  - torque saturation `0`
  - 최대 토크 약 `3.75 Nm`
- 해석:
  - `0.65` 수준의 좌측 하중 이동은 현재 모델에서 안정적으로 가능하다.
  - 그러나 오른발 lift를 위해서는 오른발 normal force를 더 낮춰야 한다.

## 테스트 1: Target Left Ratio 0.78 자세 탐색

### 실행 개념

`0.65` 안정 자세에서 시작하여 최종 left force ratio `0.78`을 목표로 하는 transition-aware pose search를 수행했다. 탐색 scoring에는 다음 항목을 포함했다.

- 목표 left/right force ratio
- 최종 roll/pitch/yaw
- 최대 roll
- qvel norm
- contact 유지
- normal force
- torque saturation
- 오른발 force 상한
- 왼발 force 하한

### 산출물

- 후보 자세:
  - `configs/right_unload_from_left065_contact_constrained_multipoint.json`
- 탐색 결과:
  - `outputs/analysis/right_unload_search_from_left065_contact_constrained_multipoint/weight_shift_transition_search_summary.json`
  - `outputs/analysis/right_unload_search_from_left065_contact_constrained_multipoint/weight_shift_transition_candidates.csv`

### 주요 결과

| 항목 | 값 |
|---|---:|
| 목표 left ratio | 0.780 |
| 최종 left ratio | 0.728 |
| 최종 right ratio | 0.272 |
| 최종 left force | 65.74 N |
| 최종 right force | 24.62 N |
| 최종 roll | 2.609 rad |
| 최대 roll | 3.141 rad |
| 최대 qvel norm | 6.036 |
| 최대 contact force | 405.72 N |
| 최대 torque | 16.00 Nm |
| torque saturation fraction | 0.0 |
| min left contacts | 0 |
| min right contacts | 0 |

### 판단

실패다.

수치상 left ratio는 `0.73` 근처까지 올라갔지만, roll이 `2.61 rad`이고 최대 roll이 거의 `pi`에 도달했다. 이는 오른발을 정상적으로 unloading한 것이 아니라, 로봇이 넘어지면서 접촉/하중 분포가 깨진 결과로 판단해야 한다.

또한 torque saturation은 발생하지 않았다. 따라서 이번 실패의 1차 원인은 모터 토크 부족이라기보다 다음 쪽에 가깝다.

- 한 발 지지로 넘어가는 동안 접촉 안정성이 유지되지 않음
- 발바닥 지지 다각형과 COM/상체 위치 관계가 불리함
- 현재 단순 PD + stabilizer 구조로는 오른발 하중 제거를 안정적으로 제어하지 못함

## 테스트 2: Target Left Ratio 0.70 보수적 자세 탐색

### 실행 개념

`0.78` 목표가 실패했기 때문에 목표를 `0.70`으로 낮췄다. 대신 roll/contact 제약을 더 강하게 두고, 오른발 접촉을 완전히 버리지 않도록 최소 right contact 조건도 유지했다.

### 산출물

- 후보 자세:
  - `configs/right_unload_left070_contact_constrained_multipoint.json`
- 탐색 결과:
  - `outputs/analysis/right_unload_search_left070_from_left065_contact_constrained_multipoint/weight_shift_transition_search_summary.json`
  - `outputs/analysis/right_unload_search_left070_from_left065_contact_constrained_multipoint/weight_shift_transition_candidates.csv`

### 주요 결과

| 항목 | 값 |
|---|---:|
| 목표 left ratio | 0.700 |
| 최종 left ratio | 0.755 |
| 최종 right ratio | 0.245 |
| 최종 left force | 68.23 N |
| 최종 right force | 22.13 N |
| 최종 roll | 2.513 rad |
| 최대 roll | 3.140 rad |
| 최대 qvel norm | 5.972 |
| 최대 contact force | 409.53 N |
| 최대 torque | 16.13 Nm |
| torque saturation fraction | 0.0 |
| min left contacts | 0 |
| min right contacts | 0 |

### 판단

실패다.

목표를 `0.70`으로 낮춰도 안정 후보가 나오지 않았다. 최종 left ratio는 목표보다 높지만, 이것 역시 안정적인 하중 이동이 아니라 roll 붕괴 이후의 결과다.

`min left/right contacts = 0`이므로 transition 중 양쪽 접촉이 모두 깨지는 구간이 있다. 이 상태는 swing foot lift 이전 단계로 사용할 수 없다.

## 테스트 3: 0.65 자세 유지 + Force Ratio Feedback

### 실행 개념

자세 탐색만으로는 안정적인 오른발 unloading 후보가 나오지 않았기 때문에, 기존에 효과가 있었던 direct force-ratio feedback을 `0.65` 안정 자세에 적용했다.

조건:

- 시작/유지 자세:
  - `configs/weight_shift_left065_contact_constrained_multipoint.json`
- force role:
  - `pitch`
- 적용 joint:
  - `ankle`
- side mode:
  - `opposite`
- gain:
  - `kforce = 5`
- 목표:
  - left ratio `0.70`
  - left ratio `0.72`

### 산출물

- `0.70` 목표:
  - `outputs/analysis/force_ratio_hold065_target070_pitch_ankle_k5_opp_pos/force_ratio_sequence_render.mp4`
  - `outputs/analysis/force_ratio_hold065_target070_pitch_ankle_k5_opp_pos/force_ratio_sequence_render.gif`
  - `outputs/analysis/force_ratio_hold065_target070_pitch_ankle_k5_opp_pos/force_ratio_sequence_timeline.csv`
- `0.72` 목표:
  - `outputs/analysis/force_ratio_hold065_target072_pitch_ankle_k5_opp_pos/force_ratio_sequence_render.mp4`
  - `outputs/analysis/force_ratio_hold065_target072_pitch_ankle_k5_opp_pos/force_ratio_sequence_render.gif`
  - `outputs/analysis/force_ratio_hold065_target072_pitch_ankle_k5_opp_pos/force_ratio_sequence_timeline.csv`

### 주요 결과

| 목표 | 최종 left ratio | 최종 right ratio | 최종 roll | 최대 roll | 최대 torque | saturation |
|---:|---:|---:|---:|---:|---:|---:|
| 0.70 | 0.599 | 0.401 | 2.643 rad | 3.142 rad | 16.85 Nm | 0.0 |
| 0.72 | 0.597 | 0.403 | 2.643 rad | 3.142 rad | 16.81 Nm | 0.0 |

### 판단

실패다.

force feedback을 더 걸어도 left ratio가 목표로 수렴하지 않고, 오히려 roll 붕괴가 발생한다. 즉 현재 안정 `0.65` 자세 주변에서 단순 ankle pitch force feedback만으로 오른발 하중을 swing 준비 수준까지 낮추기는 어렵다.

## 종합 판단

현재까지의 검증 상태는 다음과 같다.

| 단계 | 결과 |
|---|---|
| 양발 standing | 성공 |
| 안정 weight shift `0.60` | 성공 |
| 안정 weight shift `0.62` | 성공 |
| contact-constrained weight shift `0.65` | 성공 |
| 오른발 unload `0.70+` | 실패 |
| 오른발 swing foot lift 준비 | 아직 불가 |

중요한 점은 이번 실패가 최대 토크 부족으로 보이지 않는다는 것이다.

- 최대 토크는 약 `16~17 Nm` 수준이다.
- 설정 torque limit `30 Nm` 대비 saturation은 `0.0`이다.
- 따라서 “모터가 약해서 못 버틴다”는 결론을 내리면 안 된다.

현재 더 유력한 병목은 다음이다.

1. 발바닥 접촉 모델/실제 발 footprint가 한 발 지지에 충분한 안정 여유를 주는지 불확실하다.
2. 상체/base 질량 중심 위치가 한 발 지지 시 지지 발 안쪽으로 충분히 들어오지 못한다.
3. ankle roll/pitch와 hip roll/pitch의 축/부호/lever arm이 한 발 지지 제어에 불리할 수 있다.
4. 현재 제어기는 RL이 아니라 단순 PD + 보조 stabilizer이므로, 실제 RL에서는 더 나아질 가능성은 있다. 하지만 하드웨어 검증 관점에서는 swing으로 넘어가기 전 실패 gate로 본다.

## 다음 조치

다음 단계는 바로 발을 들어 올리는 것이 아니라, 실패 원인을 더 분리해야 한다.

1. `0.65` 안정 자세의 COM 투영점이 왼발 contact polygon 내부 어디에 있는지 계산한다.
2. 왼발 단독 지지 polygon 기준 정적 안정 여유를 수치화한다.
3. base/torso 질량 중심을 좌측으로 이동시키는 가상 설계 변형을 만들어, 같은 제어기로 `0.70+` unloading이 가능한지 확인한다.
4. 발 폭/발 길이/contact pad 위치를 가상으로 키운 모델을 만들어, 실패가 발바닥 지지면 부족인지 확인한다.
5. 위 두 변형 중 하나라도 성공하면, 하드웨어 형상 수정 방향을 좁힌다.

현재 gate는 다음과 같이 둔다.

- swing foot lift로 진행 조건:
  - left force ratio `>= 0.70`
  - right force `<= 25 N`
  - final roll `<= 0.16 rad`
  - max roll `<= 0.22 rad`
  - contact loss 없음
  - torque saturation 없음
- 현재 결과:
  - ratio/force 조건 일부는 순간적으로 만족할 수 있으나 roll/contact 조건을 통과하지 못함

