# Neutral PD Standing Probe

## Purpose

RL 학습 전, 현재 floating-base MJCF가 neutral joint pose에서 단순 joint-space PD로 버틸 수 있는지 확인한다. 이 단계는 최종 보행 가능성 판정이 아니라 standing/squat/weight-shift로 넘어가기 위한 sanity check다.

## Command

```bash
python3 scripts/probe_pd_standing.py
```

## Controller

- Target joint pose: `model.qpos0` hinge values
- Kp: `60.0`
- Kd: `4.0`
- Torque limit: `+/-100.0 Nm`
- Initial base z: `0.000000`

## Summary

- Duration: `2.0` sec
- Final base z: `0.511438`
- Final roll: `-2.331285` rad
- Final pitch: `-0.443708` rad
- Max qvel norm: `835.484923`
- Max contact normal force: `4504.513448`
- Max actuator torque observed: `100.000000` Nm

## Outputs

- CSV: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_user_size_mass_contact_zero_basez_exact000/neutral_pd_standing.csv`
- JSON: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_user_size_mass_contact_zero_basez_exact000/neutral_pd_standing_summary.json`

## Interpretation

Neutral pose PD가 안정적이지 않으면 바로 RL로 넘어가지 않는다. 먼저 neutral standing pose, foot collision, base height, joint axis, mass distribution을 점검해야 한다.
