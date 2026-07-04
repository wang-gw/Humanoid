# STEP contact config 적용 모델 PD Probe

## 목적

`URDF_F_link_step_contact.xml`은 STEP 기반 foot contact 초안 JSON을 적용해 만든 반복 검증용 모델이다. 이 probe는 contact config 적용 파이프라인이 정상 동작하는지, 그리고 기존 pose 후보가 이 모델에서 standing을 통과하는지 확인한다.

## 실행 명령

```bash
python3 scripts/probe_pd_standing.py --model envs/robots/urdf_f_link/URDF_F_link_step_contact.xml --pose-json configs/standing_pose_candidate_named.json --base-z 0.06964194138592063 --out-dir outputs/analysis/pd_standing_link_step_contact_pose_candidate_basez --doc-dir docs/hardware_validation --doc-name 36_step_contact_pose_pd_standing_probe.md
python3 scripts/plot_pd_standing.py --csv outputs/analysis/pd_standing_link_step_contact_pose_candidate_basez/neutral_pd_standing.csv --out-dir outputs/analysis/pd_standing_link_step_contact_pose_candidate_basez
```

## 조건

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_step_contact.xml`
- contact config: `/home/king0519/projects/Humanoid/configs/prefilled/foot_contact_prefilled_from_step_body_centered.json`
- 목표 joint pose: `configs/standing_pose_candidate_named.json`
- base z: `0.069642`
- Kp: `60.0`
- Kd: `4.0`
- torque limit: `+/-100.0 Nm`

## 결과

- 최종 base z: `1.271381`
- 최종 roll: `-2.088366` rad
- 최종 pitch: `-1.093511` rad
- 최대 qvel norm: `959.736245`
- 최대 contact normal force: `3930.414182`
- 관측 최대 actuator torque: `100.000000` Nm

## 조인트별 토크 요약

| actuator | 최대 절대 토크 Nm | RMS 토크 Nm |
| --- | ---: | ---: |
| `motor_left_hip_roll` | 83.619850 | 32.798480 |
| `motor_left_hip_pitch` | 100.000000 | 34.045973 |
| `motor_left_knee_pitch` | 100.000000 | 34.995359 |
| `motor_left_ankle_pitch` | 100.000000 | 96.643391 |
| `motor_left_ankle_roll` | 100.000000 | 97.220194 |
| `motor_right_hip_roll` | 100.000000 | 44.934355 |
| `motor_right_hip_pitch` | 100.000000 | 59.536192 |
| `motor_right_knee_pitch` | 97.715003 | 34.998689 |
| `motor_right_ankle_pitch` | 100.000000 | 96.403683 |
| `motor_right_ankle_roll` | 100.000000 | 95.823416 |

## 산출물

- CSV: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_link_step_contact_pose_candidate_basez/neutral_pd_standing.csv`
- JSON: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_link_step_contact_pose_candidate_basez/neutral_pd_standing_summary.json`
- Plot: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_link_step_contact_pose_candidate_basez/neutral_pd_standing_plot.png`

## 해석

STEP 기반 contact config 적용 모델은 MuJoCo 로드와 COM/support 분석을 통과했다. 하지만 기존 pose 후보로는 여전히 안정 standing을 만들지 못한다.

따라서 contact config 적용 파이프라인은 준비됐지만, RL 또는 최종 모터 선정으로 넘어갈 수는 없다. 다음 gate는 실제 standing pose, link mass/inertia, actuator 후보 사양을 채워 넣는 것이다.
