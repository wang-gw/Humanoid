# Roll Free Impulse Response

## 목적

contact와 중력 영향을 제거한 상태에서 roll actuator torque sign과 link 회전 방향을 확인한다. 이 실험은 roll contact 지지 문제와 joint axis/sign 문제를 분리하기 위한 것이다.

## 실행 명령

```bash
python3 scripts/audit_roll_free_impulse_response.py
```

## 조건

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml`
- pose: `/home/king0519/projects/Humanoid/configs/quasistatic_standing_pose_inertia_direct_nobase.json`
- gravity: `[0, 0, 0]`
- base z: `1.0`
- impulse torque: `25.0 Nm`
- impulse duration: `0.08 s`

## 결과

### Large Impulse

| actuator | joint | sign | delta joint q | delta base roll | L foot delta roll | R foot delta roll | max qvel |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `motor_left_hip_roll` | `left_hip_roll` | +1 | 1.432630 | 0.114697 | 3.150020 | -0.561198 | 43.937279 |
| `motor_left_hip_roll` | `left_hip_roll` | -1 | -1.377949 | 0.338970 | 0.111134 | -0.492736 | 41.637497 |
| `motor_left_ankle_roll` | `left_ankle_roll` | +1 | 0.165408 | -0.007682 | -1.479740 | -0.060917 | 183.009705 |
| `motor_left_ankle_roll` | `left_ankle_roll` | -1 | -0.438025 | -0.005302 | 1.615619 | -0.010310 | 173.138535 |
| `motor_right_hip_roll` | `right_hip_roll` | +1 | 1.402986 | 0.244237 | 0.275788 | 0.357462 | 29.965767 |
| `motor_right_hip_roll` | `right_hip_roll` | -1 | -1.857473 | 0.293686 | 0.417561 | 2.395535 | 22.282636 |
| `motor_right_ankle_roll` | `right_ankle_roll` | +1 | 1.044741 | -0.011597 | -0.029515 | -1.826802 | 201.395705 |
| `motor_right_ankle_roll` | `right_ankle_roll` | -1 | -0.817752 | -0.005323 | -0.150100 | -0.585422 | 178.823733 |

large impulse는 관절각 변화가 너무 커서 body roll 값에 wrap/비선형 효과가 섞인다. 따라서 sign 판정은 아래 small-signal 결과를 우선한다.

### Small-Signal Impulse

추가 실행 조건:

- impulse torque: `5 Nm`
- impulse duration: `0.03 s`
- total duration: `0.12 s`

| actuator | joint | sign | delta joint q | delta base roll | L foot delta roll | R foot delta roll | max qvel |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `motor_left_hip_roll` | `left_hip_roll` | +1 | 0.073110 | -0.001044 | -0.044184 | -0.028364 | 1.764580 |
| `motor_left_hip_roll` | `left_hip_roll` | -1 | -0.072987 | 0.001615 | 0.050572 | 0.028287 | 1.923336 |
| `motor_left_ankle_roll` | `left_ankle_roll` | +1 | 1.708453 | 0.001475 | 2.854579 | 0.010519 | 88.770179 |
| `motor_left_ankle_roll` | `left_ankle_roll` | -1 | -1.615223 | -0.001437 | -1.322397 | -0.016131 | 82.959116 |
| `motor_right_hip_roll` | `right_hip_roll` | +1 | 0.065872 | -0.000178 | -0.014130 | -0.019006 | 1.272108 |
| `motor_right_hip_roll` | `right_hip_roll` | -1 | -0.066013 | 0.000715 | 0.014378 | 0.023513 | 1.231266 |
| `motor_right_ankle_roll` | `right_ankle_roll` | +1 | 1.940512 | 0.002174 | 0.037522 | 2.821049 | 75.840119 |
| `motor_right_ankle_roll` | `right_ankle_roll` | -1 | -1.446895 | -0.000900 | -0.010420 | -0.251069 | 67.077549 |

Small-signal 조건에서는 모든 roll actuator가 `+/-` torque에 대해 `delta_joint_q`, `delta_base_roll`, foot roll을 반대 부호로 만든다.

## 산출물

- time-series CSV: `/home/king0519/projects/Humanoid/outputs/analysis/roll_free_impulse_response/roll_free_impulse_response.csv`
- summary CSV: `/home/king0519/projects/Humanoid/outputs/analysis/roll_free_impulse_response/roll_free_impulse_response_summary.csv`
- summary JSON: `/home/king0519/projects/Humanoid/outputs/analysis/roll_free_impulse_response/roll_free_impulse_response_summary.json`
- small-signal time-series CSV: `/home/king0519/projects/Humanoid/outputs/analysis/roll_free_impulse_response_small/roll_free_impulse_response.csv`
- small-signal summary CSV: `/home/king0519/projects/Humanoid/outputs/analysis/roll_free_impulse_response_small/roll_free_impulse_response_summary.csv`
- small-signal summary JSON: `/home/king0519/projects/Humanoid/outputs/analysis/roll_free_impulse_response_small/roll_free_impulse_response_summary.json`

## 판단 기준

같은 actuator에서 `+`와 `-` impulse의 `delta_joint_q`가 명확히 반대 부호면 actuator-to-joint sign은 MuJoCo 내부에서 정상적으로 작동한다. 이때 contact 실험에서 반대 방향 roll이 나오지 않는다면, 우선 원인은 contact/초기자세/관성 모델 쪽으로 본다.

## 판단

MuJoCo actuator-to-joint sign 자체는 정상으로 본다. Small-signal free impulse에서 네 roll actuator 모두 `+/-` torque에 대해 관절각과 body roll이 반대 방향으로 반응했다.

따라서 57번 contact 포함 roll impulse에서 `+/-`가 명확히 반대 방향 roll을 만들지 못한 원인은 roll joint sign 하나로 설명하기 어렵다. 우선순위는 다음으로 이동한다.

1. 발바닥 contact가 양발 roll 지지 모멘트를 안정적으로 만들지 못함
2. 시작 pose에서 양발 접촉 preload가 부족하거나 비대칭임
3. ankle roll 계열의 관성/질량/감속기 가정이 너무 작아 작은 토크에도 과도하게 움직임
4. `base_link`와 hip-roll 주변 inertial frame 미확정

특히 small-signal에서도 ankle roll은 `5 Nm`, `0.03 s` impulse만으로 큰 관절각 변화를 보인다. 이는 contact가 없는 조건이긴 하지만, ankle roll 관성 또는 actuator/감속기 모델을 다시 확인할 필요가 있음을 시사한다.
