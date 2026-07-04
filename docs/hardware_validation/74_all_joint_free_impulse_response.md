# All-joint Free Impulse Response 감사

## 목적

73번 gravity bias torque 감사 이후, standing 실패가 actuator sign 오류 때문인지 확인한다.

이번 단계에서는 contact와 중력을 제거하고, 10개 actuator 전체에 작은 torque impulse를 넣어 다음을 확인했다.

1. `+` 토크와 `-` 토크가 joint angle을 반대 방향으로 움직이는가?
2. 각 joint의 실제 MuJoCo dominant axis role은 무엇인가?
3. 어떤 joint가 작은 토크에도 과도하게 움직이는가?

## 입력

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml`
- pose: `/home/king0519/projects/Humanoid/configs/quasistatic_standing_pose_toeheel_z_step_visual_contact.json`
- 스크립트: `/home/king0519/projects/Humanoid/scripts/audit_all_joint_free_impulse_response.py`

## 조건

| 항목 | 값 |
| --- | ---: |
| gravity | `[0, 0, 0]` |
| base z | `1.0 m` |
| total duration | `0.12 s` |
| impulse start | `0.02 s` |
| impulse duration | `0.03 s` |
| impulse torque | `0.2 Nm` |

contact와 중력을 제거했기 때문에, 이 결과는 "서기 성공/실패" 판정이 아니라 actuator-to-joint sign과 관절 민감도 감사용이다.

## 실행

```bash
python3 scripts/audit_all_joint_free_impulse_response.py --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml --pose-json configs/quasistatic_standing_pose_toeheel_z_step_visual_contact.json --out-dir outputs/analysis/all_joint_free_impulse_response_toeheel_z_step_visual_contact_small02 --impulse-torque 0.2 --impulse-duration 0.03 --duration 0.12
```

## 결과 요약

| actuator | joint | dominant role | sign check | avg abs delta joint q | + torque delta q | - torque delta q | max qvel |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: |
| `motor_left_ankle_pitch` | `left_ankle_pitch` | `roll_about_X_forward` | `OK` | `0.382509` | `0.393475` | `-0.371543` | `5.178764` |
| `motor_left_ankle_roll` | `left_ankle_roll` | `pitch_about_Y_lateral` | `OK` | `0.319370` | `0.323127` | `-0.315614` | `4.208054` |
| `motor_right_ankle_roll` | `right_ankle_roll` | `pitch_about_Y_lateral` | `OK` | `0.227513` | `0.228076` | `-0.226950` | `2.742379` |
| `motor_right_ankle_pitch` | `right_ankle_pitch` | `roll_about_X_forward` | `OK` | `0.124710` | `0.124332` | `-0.125087` | `1.552589` |
| `motor_left_knee_pitch` | `left_knee_pitch` | `roll_about_X_forward` | `OK` | `0.032574` | `0.032508` | `-0.032640` | `0.547839` |
| `motor_left_hip_pitch` | `left_hip_pitch` | `roll_about_X_forward` | `OK` | `0.013425` | `0.013429` | `-0.013422` | `0.248717` |
| `motor_right_knee_pitch` | `right_knee_pitch` | `roll_about_X_forward` | `OK` | `0.013181` | `0.013178` | `-0.013185` | `0.167761` |
| `motor_right_hip_pitch` | `right_hip_pitch` | `roll_about_X_forward` | `OK` | `0.006153` | `0.006154` | `-0.006153` | `0.108629` |
| `motor_left_hip_roll` | `left_hip_roll` | `pitch_about_Y_lateral` | `OK` | `0.002932` | `0.002932` | `-0.002932` | `0.071451` |
| `motor_right_hip_roll` | `right_hip_roll` | `pitch_about_Y_lateral` | `OK` | `0.002655` | `0.002655` | `-0.002655` | `0.050516` |

## 핵심 해석

### 1. Actuator sign 자체는 정상

10개 actuator 모두 `+` impulse와 `-` impulse에 대해 `delta_joint_q`가 반대 부호로 나왔다.

따라서 현재 standing 실패를 "MuJoCo actuator sign이 완전히 뒤집혀 있다"로 설명하기는 어렵다.

### 2. Joint 이름과 실제 역할은 계속 분리해서 봐야 함

70번 감사와 동일하게, 이름과 실제 MuJoCo 동역학 역할이 서로 다르다.

| 이름 계열 | 실제 dominant role |
| --- | --- |
| `*_hip_roll`, `*_ankle_roll` | 대부분 `pitch_about_Y_lateral` |
| `*_hip_pitch`, `*_knee_pitch`, `*_ankle_pitch` | 대부분 `roll_about_X_forward` |

따라서 RL action, stabilizer, reward 설계에서는 이름만 믿으면 안 되고, 실제 axis role 기준 매핑을 사용해야 한다.

### 3. Ankle 계열이 매우 민감함

`0.2 Nm`를 `0.03 s`만 넣었는데도 ankle 계열은 큰 joint 변위를 보였다.

- `left_ankle_pitch`: 평균 `0.3825 rad`
- `left_ankle_roll`: 평균 `0.3194 rad`
- `right_ankle_roll`: 평균 `0.2275 rad`
- `right_ankle_pitch`: 평균 `0.1247 rad`

반면 hip roll 계열은 약 `0.0027~0.0029 rad` 수준이다.

즉 ankle 모델은 hip에 비해 수십 배에서 백 배 이상 민감하게 움직인다. contact와 중력이 없는 자유공간 조건이므로 이것만으로 하드웨어 불가능 판정을 내리면 안 되지만, 현재 standing 영상에서 ankle torque가 빠르게 saturation에 들어가는 현상과 방향이 맞다.

## 판단

현재 우선 의심점은 actuator sign이 아니라 다음이다.

1. ankle link/actuator 쪽 inertia가 너무 작게 들어갔거나 frame이 잘못 들어갔을 가능성
2. actuator/감속기 rotor inertia 또는 armature가 모델에 빠져 있어 ankle이 비현실적으로 가볍게 움직이는 문제
3. joint damping/friction이 거의 없어 작은 토크에도 과도하게 가속되는 문제
4. ankle pitch/roll axis role이 CAD 의도와 MuJoCo 제어 명칭 사이에서 혼동되는 문제

## 산출물

- time-series CSV: `/home/king0519/projects/Humanoid/outputs/analysis/all_joint_free_impulse_response_toeheel_z_step_visual_contact_small02/all_joint_free_impulse_response.csv`
- summary CSV: `/home/king0519/projects/Humanoid/outputs/analysis/all_joint_free_impulse_response_toeheel_z_step_visual_contact_small02/all_joint_free_impulse_response_summary.csv`
- summary JSON: `/home/king0519/projects/Humanoid/outputs/analysis/all_joint_free_impulse_response_toeheel_z_step_visual_contact_small02/all_joint_free_impulse_response_summary.json`

## 다음 조치

1. actuator armature/damping/friction을 보수적으로 추가한 변형 모델을 만든다.
2. 특히 ankle actuator에 실제 감속기/모터 rotor inertia가 어느 정도 반영되어야 하는지 별도 입력값으로 관리한다.
3. armature/damping 변형 모델에서 standing PD와 axis-aware standing 영상을 다시 비교한다.
4. 이 비교에서 ankle 과민성이 줄고 standing 시간이 늘어나면, RL 전 모델링 보정 항목으로 확정한다.
