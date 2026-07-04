# link STL named 모델 Pose 후보 PD Standing Probe

## 목적

새로 만든 `URDF_F_link_named.xml`이 MuJoCo에서 로드되는 것만으로는 하드웨어/RL 가능성을 판단할 수 없다. 기존 named 모델에서 사용했던 pose 후보를 같은 조건으로 적용해, 새 link STL 모델에서도 동역학 결과가 어떻게 나오는지 확인한다.

이 테스트는 최종 standing controller가 아니다. 파일 매핑 후에도 기존 실패 현상이 재현되는지, 토크 로그가 새 모델 기준으로 정상 생성되는지 확인하는 단계다.

## 실행 명령

```bash
python3 scripts/probe_pd_standing.py --model envs/robots/urdf_f_link/URDF_F_link_named.xml --pose-json configs/standing_pose_candidate_named.json --out-dir outputs/analysis/pd_standing_link_named_pose_candidate --doc-dir docs/hardware_validation --doc-name 26_link_named_pose_pd_standing_probe.md
```

## 제어 조건

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_named.xml`
- 목표 joint pose: `configs/standing_pose_candidate_named.json`
- Kp: `60.0`
- Kd: `4.0`
- torque limit: `+/-100.0 Nm`
- 초기 base z: `0.042758`

## 요약 결과

- 시뮬레이션 시간: `2.0` sec
- 최종 base z: `3.673125`
- 최종 roll: `-1.077786` rad
- 최종 pitch: `-0.238622` rad
- 최대 qvel norm: `915.532495`
- 최대 contact normal force: `2071.996429`
- 관측 최대 actuator torque: `100.000000` Nm

## 조인트별 토크 요약

| actuator | 최대 절대 토크 Nm | RMS 토크 Nm |
| --- | ---: | ---: |
| `motor_left_hip_roll` | 100.000000 | 38.308104 |
| `motor_left_hip_pitch` | 100.000000 | 32.797691 |
| `motor_left_knee_pitch` | 96.561879 | 39.118926 |
| `motor_left_ankle_pitch` | 100.000000 | 96.632535 |
| `motor_left_ankle_roll` | 100.000000 | 97.288747 |
| `motor_right_hip_roll` | 100.000000 | 57.039012 |
| `motor_right_hip_pitch` | 98.799018 | 28.687309 |
| `motor_right_knee_pitch` | 100.000000 | 32.343315 |
| `motor_right_ankle_pitch` | 100.000000 | 96.562552 |
| `motor_right_ankle_roll` | 100.000000 | 94.029756 |

## 산출물

- CSV: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_link_named_pose_candidate/neutral_pd_standing.csv`
- JSON: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_link_named_pose_candidate/neutral_pd_standing_summary.json`
- Plot: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_link_named_pose_candidate/neutral_pd_standing_plot.png`

## 해석

새 link STL 모델에서도 pose 후보는 안정적인 standing을 만들지 못했다. 대부분의 actuator가 `+/-100 Nm` 임시 제한에 닿았고, ankle pitch/roll 계열은 RMS 토크도 제한에 매우 가깝다.

이 결과를 모터 선정값으로 사용하면 안 된다. 현재 결과에는 실제 standing pose 미확정, foot contact primitive 임시값, 좌우 질량 차이, joint axis/branch 확인 필요성이 섞여 있다. 따라서 다음 단계는 더 큰 모터를 고르는 것이 아니라 CAD 기준 pose/contact/mass를 확정한 뒤 같은 probe를 다시 실행하는 것이다.
