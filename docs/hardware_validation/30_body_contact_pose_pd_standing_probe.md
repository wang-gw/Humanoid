# Body-centered contact PD Standing Probe

## 목적

`URDF_F_link_body_contact.xml`은 좌우 발바닥 contact가 같은 위치에 겹치는 문제를 분리하기 위한 진단용 모델이다. 이 probe는 기존 pose 후보를 그대로 적용했을 때, foot body 중심 contact가 동역학 결과를 얼마나 개선하는지 확인한다.

## 실행 명령

```bash
python3 scripts/probe_pd_standing.py --model envs/robots/urdf_f_link/URDF_F_link_body_contact.xml --pose-json configs/standing_pose_candidate_named.json --out-dir outputs/analysis/pd_standing_link_body_contact_pose_candidate --doc-dir docs/hardware_validation --doc-name 30_body_contact_pose_pd_standing_probe.md
python3 scripts/plot_pd_standing.py --csv outputs/analysis/pd_standing_link_body_contact_pose_candidate/neutral_pd_standing.csv --out-dir outputs/analysis/pd_standing_link_body_contact_pose_candidate
```

## 조건

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_body_contact.xml`
- 목표 joint pose: `configs/standing_pose_candidate_named.json`
- base z: `0.042758`
- Kp: `60.0`
- Kd: `4.0`
- torque limit: `+/-100.0 Nm`

## 결과

- 최종 base z: `0.507482`
- 최종 roll: `-0.956487` rad
- 최종 pitch: `1.087000` rad
- 최대 qvel norm: `825.223527`
- 최대 contact normal force: `3755.480541`
- 관측 최대 actuator torque: `100.000000` Nm

## 산출물

- CSV: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_link_body_contact_pose_candidate/neutral_pd_standing.csv`
- JSON: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_link_body_contact_pose_candidate/neutral_pd_standing_summary.json`
- Plot: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_link_body_contact_pose_candidate/neutral_pd_standing_plot.png`

## 해석

발바닥 contact를 foot body 중심으로 옮기면 support 영역은 개선되지만, 기존 pose 후보와 PD gain으로는 여전히 안정 standing이 되지 않는다. 이 실행은 pose JSON에 들어 있던 기존 base z를 그대로 사용했기 때문에, 31번에서 새 contact 높이에 맞춘 base z로 다시 실행한다.
