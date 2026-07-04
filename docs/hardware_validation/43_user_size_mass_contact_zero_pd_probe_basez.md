# Frame-normalized user-size contact base-z 보정 PD Probe

## 목적

41번 후보 감사에서 선택한 `body_centered_user_size_bottom_40mm` contact와 사용자 mass를 반영한 모델에서, 0 rad standing pose가 동역학적으로 유지되는지 확인한다. base z는 geometry 분석에서 계산된 `0.057284`를 사용한다.

## 실행 명령

```bash
python3 scripts/probe_pd_standing.py --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml --base-z 0.05728416147400483 --out-dir outputs/analysis/pd_standing_user_size_mass_contact_zero_basez --doc-dir docs/hardware_validation --doc-name 43_user_size_mass_contact_zero_pd_probe_basez.md
python3 scripts/plot_pd_standing.py --csv outputs/analysis/pd_standing_user_size_mass_contact_zero_basez/neutral_pd_standing.csv --out-dir outputs/analysis/pd_standing_user_size_mass_contact_zero_basez
```

## 조건

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml`
- 목표 joint pose: 전체 revolute joint `0.0 rad`
- base z: `0.057284`
- 총질량: `9.211400 kg`
- Kp: `60.0`
- Kd: `4.0`
- torque limit: `+/-100.0 Nm`

## Geometry 선행 결과

- left sole world pos: `[0.067, -0.0215, 0.077284]`
- right sole world pos: `[-0.087, -0.0215, 0.077284]`
- 좌우 contact 높이 차이: `0.0 m`
- COM inside support x: `True`
- COM inside support y: `True`

## PD 결과

- 최종 base z: `2.912016`
- 최종 roll: `0.421224` rad
- 최종 pitch: `-0.464444` rad
- 최대 qvel norm: `791.293011`
- 최대 contact normal force: `5696.328774`
- 관측 최대 actuator torque: `100.000000` Nm

## 조인트별 토크 요약

| actuator | 최대 절대 토크 Nm | RMS 토크 Nm |
| --- | ---: | ---: |
| `motor_left_hip_roll` | 96.856597 | 34.084392 |
| `motor_left_hip_pitch` | 66.534564 | 18.675460 |
| `motor_left_knee_pitch` | 99.120450 | 25.141906 |
| `motor_left_ankle_pitch` | 100.000000 | 94.680263 |
| `motor_left_ankle_roll` | 100.000000 | 95.707099 |
| `motor_right_hip_roll` | 57.233165 | 20.845637 |
| `motor_right_hip_pitch` | 85.809472 | 31.254149 |
| `motor_right_knee_pitch` | 100.000000 | 32.524394 |
| `motor_right_ankle_pitch` | 100.000000 | 96.865001 |
| `motor_right_ankle_roll` | 100.000000 | 96.723419 |

## 산출물

- CSV: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_user_size_mass_contact_zero_basez/neutral_pd_standing.csv`
- JSON: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_user_size_mass_contact_zero_basez/neutral_pd_standing_summary.json`
- Plot: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_user_size_mass_contact_zero_basez/neutral_pd_standing_plot.png`

## 해석

좌우 contact 높이와 COM/support 조건을 맞춰도 0 rad pose는 안정적인 PD standing을 만들지 못했다. 따라서 현재 실패는 contact frame 문제만으로 설명되지 않는다.

다음 분리 대상은 joint axis/action sign과 actuator torque 한계다. 특히 ankle pitch/roll 계열은 RMS torque가 `94-97 Nm` 수준으로 계속 `100 Nm` 제한에 가깝다.
