# Roll Impulse Response Audit

## 목적

standing 실패가 roll 방향에서 지속되므로, 각 roll actuator에 짧은 torque impulse를 넣어 base roll과 좌우 foot contact force가 어떤 방향으로 반응하는지 확인한다.

## 실행 명령

```bash
python3 scripts/audit_roll_impulse_response.py
```

## 입력

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml`
- pose: `/home/king0519/projects/Humanoid/configs/quasistatic_standing_pose_inertia_direct_nobase.json`
- impulse torque: `35.0 Nm`
- impulse duration: `0.06 s`
- torque limit: `100.0 Nm`

## 결과

### Clearance 조건

처음 실행은 pose 파일의 `base_z`를 그대로 사용했다. 이 pose는 발바닥에 약 `5 mm` clearance가 있어 초기 접촉력이 거의 없으므로, 낙하/접촉 전이가 impulse 응답에 섞였다.

| actuator | sign | final roll | delta roll from impulse start | final pitch | max qvel | max contact force | final L/R contact force |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `motor_left_hip_roll` | +1 | 0.391888 | 0.371801 | 0.120222 | 206.560636 | 2805.058046 | `618.576 / 0.000` |
| `motor_left_hip_roll` | -1 | 0.596920 | 0.576833 | -0.194259 | 206.560636 | 2805.058046 | `0.000 / 602.663` |
| `motor_left_ankle_roll` | +1 | 0.459043 | 0.438956 | -0.163675 | 206.560636 | 2805.058046 | `0.000 / 0.000` |
| `motor_left_ankle_roll` | -1 | 0.441408 | 0.421320 | 0.053360 | 206.560636 | 2897.202775 | `518.097 / 0.000` |
| `motor_right_hip_roll` | +1 | 0.422073 | 0.401986 | -0.225365 | 206.560636 | 2805.058046 | `0.000 / 629.697` |
| `motor_right_hip_roll` | -1 | 0.533055 | 0.512968 | -0.305415 | 206.560636 | 2805.058046 | `0.000 / 0.000` |
| `motor_right_ankle_roll` | +1 | 0.904152 | 0.884065 | -0.516090 | 206.560636 | 2805.058046 | `0.000 / 0.000` |
| `motor_right_ankle_roll` | -1 | 0.659195 | 0.639108 | -0.548677 | 206.560636 | 2805.058046 | `0.000 / 0.000` |

### 양발 contact penetration 조건

두 번째 기준 실행은 `base_z_offset=-0.006 m`를 적용해 양발이 작은 관통 여유를 가지고 접촉하도록 했다. 이 조건에서도 no-impulse baseline 자체가 짧은 시간 안에 `+roll` 방향으로 drift한다.

No-impulse baseline:

- final roll: `0.167927 rad`
- delta roll from impulse start: `0.167385 rad`

Drift 보정 후 impulse 효과:

| actuator | sign | raw delta roll | effect vs no-impulse |
| --- | ---: | ---: | ---: |
| `motor_left_hip_roll` | +1 | 0.182844 | 0.015459 |
| `motor_left_hip_roll` | -1 | 0.174184 | 0.006799 |
| `motor_left_ankle_roll` | +1 | 0.189919 | 0.022534 |
| `motor_left_ankle_roll` | -1 | 0.186058 | 0.018672 |
| `motor_right_hip_roll` | +1 | 0.170763 | 0.003377 |
| `motor_right_hip_roll` | -1 | 0.170398 | 0.003013 |
| `motor_right_ankle_roll` | +1 | 0.225528 | 0.058142 |
| `motor_right_ankle_roll` | -1 | 0.162075 | -0.005310 |

## 산출물

- time-series CSV: `/home/king0519/projects/Humanoid/outputs/analysis/roll_impulse_response/roll_impulse_response.csv`
- summary CSV: `/home/king0519/projects/Humanoid/outputs/analysis/roll_impulse_response/roll_impulse_response_summary.csv`
- summary JSON: `/home/king0519/projects/Humanoid/outputs/analysis/roll_impulse_response/roll_impulse_response_summary.json`
- contact-start CSV: `/home/king0519/projects/Humanoid/outputs/analysis/roll_impulse_response_contact_start/roll_impulse_response.csv`
- both-feet penetration CSV: `/home/king0519/projects/Humanoid/outputs/analysis/roll_impulse_response_both_feet_penetrating/roll_impulse_response.csv`
- no-impulse baseline CSV: `/home/king0519/projects/Humanoid/outputs/analysis/roll_impulse_response_no_impulse_baseline/roll_impulse_response.csv`

## 판단 기준

같은 actuator에서 `+`와 `-` impulse가 base roll을 명확히 반대 방향으로 움직이면 torque sign 관측은 정상이다. 두 방향이 모두 같은 쪽으로 무너지거나 contact force가 한쪽으로 급격히 사라지면, roll contact support 또는 joint axis/sign 모델을 추가로 확인해야 한다.

## 판단

현재 모델에서는 roll impulse의 `+/-` 부호가 base roll을 명확히 반대 방향으로 만들지 못한다. 하지만 이것을 곧바로 joint sign 오류라고 단정하면 안 된다. no-impulse baseline만으로도 `+roll` drift가 크고, 대부분의 impulse 효과는 이 drift에 비해 작다.

더 중요한 관찰은 contact 쪽이다.

- clearance pose에서는 초기 접촉이 거의 없어 낙하/충돌이 먼저 발생한다.
- `base_z_offset=-0.005463...`에서는 왼발 하단이 정확히 0이라 MuJoCo contact pair가 생성되지 않고 오른발만 접촉한다.
- `base_z_offset=-0.006`으로 양발에 작은 관통 여유를 줘도, 양발 contact가 동시에 유지되는 sample은 매우 적다.

따라서 현재 roll 실패의 우선 원인은 roll actuator sign 하나라기보다, 양발 contact patch가 안정적인 roll 지지 모멘트를 만들지 못하는 문제로 보는 것이 더 타당하다.

다음 단계는 foot contact를 단일 box 하나가 아니라 발의 네 모서리/패드 contact로 나눈 variant를 만들어 roll 지지 모멘트가 개선되는지 확인하는 것이다.
