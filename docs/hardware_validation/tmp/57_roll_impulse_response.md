# Roll Impulse Response Audit

## 목적

standing 실패가 roll 방향에서 지속되므로, 각 roll actuator에 짧은 torque impulse를 넣어 base roll과 좌우 foot contact force가 어떤 방향으로 반응하는지 확인한다.

## 실행 명령

```bash
python3 scripts/audit_roll_impulse_response.py
```

## 입력

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded.xml`
- pose: `/home/king0519/projects/Humanoid/configs/quasistatic_standing_pose_inertia_direct_nobase.json`
- impulse torque: `25.0 Nm`
- impulse duration: `0.04 s`
- torque limit: `100.0 Nm`
- base z offset: `-0.006 m`

## 결과

| actuator | sign | final roll | delta roll from impulse start | final pitch | max qvel | max contact force | final L/R contact force |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `motor_left_hip_roll` | +1 | 0.052876 | 0.051857 | 0.224916 | 203.127680 | 1821.687305 | `0.000 / 0.000` |
| `motor_left_hip_roll` | -1 | 0.042063 | 0.041044 | 0.218505 | 202.058253 | 1814.310381 | `0.000 / 0.000` |
| `motor_left_ankle_roll` | +1 | 0.047285 | 0.046266 | 0.221601 | 202.591354 | 1817.998843 | `0.000 / 0.000` |
| `motor_left_ankle_roll` | -1 | 0.052143 | 0.051124 | 0.216526 | 202.591354 | 1817.998843 | `0.000 / 0.000` |
| `motor_right_hip_roll` | +1 | 0.052835 | 0.051816 | 0.236742 | 202.822390 | 1802.521012 | `0.000 / 0.000` |
| `motor_right_hip_roll` | -1 | 0.043260 | 0.042241 | 0.204803 | 202.363049 | 1833.476674 | `0.000 / 0.000` |
| `motor_right_ankle_roll` | +1 | 0.062269 | 0.061250 | 0.260638 | 201.495647 | 1920.350321 | `0.000 / 0.000` |
| `motor_right_ankle_roll` | -1 | 0.048150 | 0.047131 | 0.199733 | 204.024387 | 1814.109224 | `0.000 / 0.000` |

## 산출물

- time-series CSV: `/home/king0519/projects/Humanoid/outputs/analysis/roll_impulse_response_user_pad_toeheel_z_grounded/roll_impulse_response.csv`
- summary CSV: `/home/king0519/projects/Humanoid/outputs/analysis/roll_impulse_response_user_pad_toeheel_z_grounded/roll_impulse_response_summary.csv`
- summary JSON: `/home/king0519/projects/Humanoid/outputs/analysis/roll_impulse_response_user_pad_toeheel_z_grounded/roll_impulse_response_summary.json`

## 판단 기준

같은 actuator에서 `+`와 `-` impulse가 base roll을 명확히 반대 방향으로 움직이면 torque sign 관측은 정상이다. 두 방향이 모두 같은 쪽으로 무너지거나 contact force가 한쪽으로 급격히 사라지면, roll contact support 또는 joint axis/sign 모델을 추가로 확인해야 한다.
