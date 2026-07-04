# Named 모델 Pose 후보 PD Standing Probe

## 목적

읽기 쉬운 joint/actuator 이름을 적용한 named 모델에서 pose 후보가 PD standing을 통과하는지 확인한다. 이 단계는 이름 변경 후에도 기존 실패가 동일하게 재현되는지, 그리고 torque log가 사람이 읽을 수 있는 이름으로 남는지 확인하기 위한 것이다.

## 실행 명령

```bash
python3 scripts/probe_pd_standing.py --model envs/robots/urdf_f/URDF_F_named.xml --pose-json configs/standing_pose_candidate_named.json --out-dir outputs/analysis/pd_standing_named_pose_candidate --doc-name 15_named_pose_pd_standing_probe.md
```

## 제어기

- 목표 joint pose: `configs/standing_pose_candidate_named.json`의 hinge target
- Kp: `60.0`
- Kd: `4.0`
- torque limit: `+/-100.0 Nm`
- 초기 base z: `0.042758`

## 요약

- 실행 시간: `2.0` sec
- 최종 base z: `3.673125`
- 최종 roll: `-1.077786` rad
- 최종 pitch: `-0.238622` rad
- 최대 qvel norm: `915.532495`
- 최대 contact normal force: `2071.996429`
- 관측된 최대 actuator torque: `100.000000` Nm

## 산출물

- CSV: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_named_pose_candidate/neutral_pd_standing.csv`
- JSON: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_named_pose_candidate/neutral_pd_standing_summary.json`
- Plot: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_named_pose_candidate/neutral_pd_standing_plot.png`

## 해석

named 모델에서도 pose 후보는 안정적인 standing을 만들지 못했다. 다만 CSV torque column이 `tau_motor_left_knee_pitch`처럼 기록되므로, 앞으로 토크 분석과 action order 검증은 named 모델 기준으로 진행하는 것이 낫다.
