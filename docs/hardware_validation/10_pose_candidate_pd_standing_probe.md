# Pose 후보 PD Standing Probe

## 목적

RL 학습 전, 현재 floating-base MJCF가 neutral joint pose에서 단순 joint-space PD로 버틸 수 있는지 확인한다. 이 단계는 최종 보행 가능성 판정이 아니라 standing/squat/weight-shift로 넘어가기 위한 sanity check다.

## 실행 명령

```bash
python3 scripts/probe_pd_standing.py
```

## 제어기

- 목표 joint pose: `configs/standing_pose_candidate.json`의 hinge target
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

- CSV: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_pose_candidate/neutral_pd_standing.csv`
- JSON: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_pose_candidate/neutral_pd_standing_summary.json`

## 해석

Neutral pose PD가 안정적이지 않으면 바로 RL로 넘어가지 않는다. 먼저 neutral standing pose, foot collision, base height, joint axis, mass distribution을 점검해야 한다.
