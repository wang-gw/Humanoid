# Body-centered contact base-z 보정 PD Probe

## 목적

30번 probe는 기존 pose JSON의 base z를 그대로 사용했다. 새 body-centered contact 모델에서는 중립 기하 분석에서 계산된 base z가 `0.069642`이므로, 같은 pose 후보를 더 일관된 초기 높이에서 다시 검증한다.

## 실행 명령

```bash
python3 scripts/probe_pd_standing.py --model envs/robots/urdf_f_link/URDF_F_link_body_contact.xml --pose-json configs/standing_pose_candidate_named.json --base-z 0.06964194138592063 --out-dir outputs/analysis/pd_standing_link_body_contact_pose_candidate_basez --doc-dir docs/hardware_validation --doc-name 31_body_contact_pose_pd_standing_probe_basez.md
python3 scripts/plot_pd_standing.py --csv outputs/analysis/pd_standing_link_body_contact_pose_candidate_basez/neutral_pd_standing.csv --out-dir outputs/analysis/pd_standing_link_body_contact_pose_candidate_basez
```

## 조건

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_body_contact.xml`
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

- CSV: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_link_body_contact_pose_candidate_basez/neutral_pd_standing.csv`
- JSON: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_link_body_contact_pose_candidate_basez/neutral_pd_standing_summary.json`
- Plot: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_link_body_contact_pose_candidate_basez/neutral_pd_standing_plot.png`

## 해석

base z를 새 contact 모델에 맞춰도 안정 standing은 되지 않는다. 발바닥 contact 겹침은 반드시 수정해야 하는 모델 문제지만, 그것만으로는 현재 pose 후보와 actuator/controller 가정이 충분하다고 볼 수 없다.

특히 ankle pitch/roll 계열 RMS 토크가 여전히 `95-97 Nm` 수준으로 임시 제한에 거의 붙어 있다. 다음 검증은 실제 CAD standing pose와 실제 foot contact origin을 확정한 뒤 수행해야 한다.
