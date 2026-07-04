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
- impulse torque: `5.0 Nm`
- impulse duration: `0.03 s`

## 결과

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

## 산출물

- time-series CSV: `/home/king0519/projects/Humanoid/outputs/analysis/roll_free_impulse_response_small/roll_free_impulse_response.csv`
- summary CSV: `/home/king0519/projects/Humanoid/outputs/analysis/roll_free_impulse_response_small/roll_free_impulse_response_summary.csv`
- summary JSON: `/home/king0519/projects/Humanoid/outputs/analysis/roll_free_impulse_response_small/roll_free_impulse_response_summary.json`

## 판단 기준

같은 actuator에서 `+`와 `-` impulse의 `delta_joint_q`가 명확히 반대 부호면 actuator-to-joint sign은 MuJoCo 내부에서 정상적으로 작동한다. 이때 contact 실험에서 반대 방향 roll이 나오지 않는다면, 우선 원인은 contact/초기자세/관성 모델 쪽으로 본다.
