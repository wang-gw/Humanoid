# Gravity Bias Torque 감사

## 목적

현재 standing 실패가 단순히 "정적 하중을 버티는 데 필요한 토크가 너무 커서" 발생하는지 1차로 확인한다.

이 단계에서는 MuJoCo의 `qfrc_bias`를 사용해 현재 pose에서 zero velocity/zero acceleration일 때 각 hinge joint에 걸리는 중력 bias 토크를 계산했다.

주의: 이 값은 free-base 상태의 gravity-bias 감사이며, 발 접촉 제약을 포함한 완전한 constrained inverse dynamics 결과는 아니다. 하지만 링크 질량/COM이 joint에 만드는 기본 중력 토크 규모를 보는 1차 필터로는 유용하다.

## 입력

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml`
- pose: `/home/king0519/projects/Humanoid/configs/quasistatic_standing_pose_toeheel_z_step_visual_contact.json`
- 스크립트: `/home/king0519/projects/Humanoid/scripts/audit_gravity_bias_torque.py`

## 실행

```bash
python3 scripts/audit_gravity_bias_torque.py --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml --pose-json configs/quasistatic_standing_pose_toeheel_z_step_visual_contact.json --out-dir outputs/analysis/gravity_bias_torque_toeheel_z_step_visual_contact
```

## 결과

| joint | actuator | q rad | gravity bias torque Nm | 100 Nm 대비 |
| --- | --- | ---: | ---: | ---: |
| `left_hip_roll` | `motor_left_hip_roll` | `0.127293` | `3.672300` | `3.67%` |
| `left_hip_pitch` | `motor_left_hip_pitch` | `-0.079019` | `0.552429` | `0.55%` |
| `left_knee_pitch` | `motor_left_knee_pitch` | `-0.013438` | `0.466464` | `0.47%` |
| `left_ankle_pitch` | `motor_left_ankle_pitch` | `0.009604` | `-0.005404` | `0.01%` |
| `left_ankle_roll` | `motor_left_ankle_roll` | `-0.057963` | `-0.237529` | `0.24%` |
| `right_hip_roll` | `motor_right_hip_roll` | `0.256924` | `2.059779` | `2.06%` |
| `right_hip_pitch` | `motor_right_hip_pitch` | `0.055126` | `-0.802600` | `0.80%` |
| `right_knee_pitch` | `motor_right_knee_pitch` | `-0.047217` | `0.473511` | `0.47%` |
| `right_ankle_pitch` | `motor_right_ankle_pitch` | `-0.027527` | `0.101867` | `0.10%` |
| `right_ankle_roll` | `motor_right_ankle_roll` | `-0.253985` | `-0.243265` | `0.24%` |

최대 절대값:

| 항목 | 값 |
| --- | ---: |
| max abs gravity bias torque | `3.672300 Nm` |
| max joint | `left_hip_roll` |

## 해석

1. 정적 gravity-bias 기준으로는 어떤 joint도 `100 Nm` 근처에 가지 않는다.
2. 따라서 69~72번에서 관찰한 `100 Nm` saturation은 정지 자세를 버티는 기본 중력 토크 때문이라기보다, 넘어지는 동역학 반응, 접촉 충격, 잘못된 축/부호 대응, 또는 자세가 하중 전달에 불리한 문제일 가능성이 크다.
3. 특히 영상에서 ankle 계열 actuator가 빠르게 포화되는 것은 "정적 중력 토크가 너무 큼"이 아니라, 넘어지는 동안 PD가 자세 오차와 속도를 잡으려다 saturation에 들어가는 현상으로 보는 것이 더 자연스럽다.

## 산출물

- CSV: `/home/king0519/projects/Humanoid/outputs/analysis/gravity_bias_torque_toeheel_z_step_visual_contact/gravity_bias_torque.csv`
- JSON: `/home/king0519/projects/Humanoid/outputs/analysis/gravity_bias_torque_toeheel_z_step_visual_contact/gravity_bias_torque_summary.json`

## 현재 판단

최종 모터 선정 단계로 바로 가기에는 아직 이르다. 지금 필요한 것은 motor torque 부족 판정이 아니라, 모델의 joint axis/부호/기구학적 하중 전달이 맞는지 확인하는 것이다.

다음 단계에서는 각 joint에 작은 torque impulse를 직접 넣고, 로봇 base roll/pitch와 발 접촉이 어떤 방향으로 반응하는지 확인한다. 이 결과를 CAD 의도와 맞춰서 joint axis를 수정할지 결정한다.
