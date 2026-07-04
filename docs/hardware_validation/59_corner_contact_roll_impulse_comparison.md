# Corner Contact Roll Impulse Comparison

## 목적

57번 roll impulse 감사에서 단일 sole box가 roll 방향 지지 모멘트를 충분히 만들지 못하는 것으로 보였다. 따라서 58번에서 만든 네 모서리 contact pad 모델을 같은 조건으로 비교해, roll contact 유지가 개선되는지 확인한다.

## 비교 모델

| 구분 | 모델 |
| --- | --- |
| single box contact | `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml` |
| corner pad contact | `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_corner_contact.xml` |

조건:

- pose: `configs/quasistatic_standing_pose_inertia_direct_nobase.json`
- base z offset: `-0.006 m`
- impulse torque: `25 Nm`
- impulse duration: `0.04 s`
- total duration: `0.25 s`

## No-Impulse Drift 비교

| contact model | final roll | delta roll | final pitch | max qvel | max contact force |
| --- | ---: | ---: | ---: | ---: | ---: |
| single box | 0.167927 | 0.167385 | -0.022105 | 195.124686 | 2334.177866 |
| corner pad | 0.207160 | 0.206448 | -0.195561 | 196.491707 | 2329.099135 |

corner pad 모델은 no-impulse 상태에서 roll drift가 더 커졌다.

## Drift 보정 Roll Impulse 효과

| actuator | sign | single box effect | corner pad effect |
| --- | ---: | ---: | ---: |
| `motor_left_hip_roll` | +1 | 0.015459 | 0.002786 |
| `motor_left_hip_roll` | -1 | 0.006799 | 0.022419 |
| `motor_left_ankle_roll` | +1 | 0.022534 | 0.031251 |
| `motor_left_ankle_roll` | -1 | 0.018672 | 0.054149 |
| `motor_right_hip_roll` | +1 | 0.003377 | -0.002051 |
| `motor_right_hip_roll` | -1 | 0.003013 | 0.002105 |
| `motor_right_ankle_roll` | +1 | 0.058142 | 0.051880 |
| `motor_right_ankle_roll` | -1 | -0.005310 | -0.028705 |

## 양발 동시 접촉 샘플 수

각 case는 총 64개 log sample이다.

| case | single box both-contact samples | corner pad both-contact samples |
| ---: | ---: | ---: |
| 0 | 6 | 3 |
| 1 | 5 | 3 |
| 2 | 4 | 3 |
| 3 | 3 | 3 |
| 4 | 5 | 3 |
| 5 | 4 | 3 |
| 6 | 4 | 3 |
| 7 | 4 | 3 |

corner pad 모델은 양발 동시 접촉 유지도 개선하지 못했다.

## 산출물

- corner contact model: `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_corner_contact.xml`
- corner no-impulse summary: `outputs/analysis/roll_impulse_response_corner_contact_no_impulse/roll_impulse_response_summary.json`
- corner impulse summary: `outputs/analysis/roll_impulse_response_corner_contact/roll_impulse_response_summary.json`
- single box impulse summary: `outputs/analysis/roll_impulse_response_both_feet_penetrating/roll_impulse_response_summary.json`

## 판단

단순히 발바닥 contact를 네 모서리 pad로 쪼개는 것은 roll 안정성을 개선하지 않았다. 오히려 no-impulse roll drift가 커졌고, 양발 동시 접촉 sample도 줄었다.

따라서 현재 문제는 “contact point 수가 부족하다”보다는 다음 쪽에 가깝다.

1. foot contact pad의 위치/크기/두께가 실제 발바닥과 맞지 않음
2. 시작 자세에서 양발이 동시에 충분히 눌리지 않음
3. roll 방향 joint axis/sign 또는 inertial frame이 아직 확정되지 않음
4. 단순 penalty contact 설정으로는 실제 고무 패드/넓은 발바닥 지지를 충분히 재현하지 못함

다음 단계는 임의 pad 분할보다 먼저 CAD 기준으로 실제 발바닥 네 모서리 좌표와 foot link frame을 확정해야 한다. 그 전까지 corner pad 모델은 최종 검증 모델로 사용하지 않는다.

