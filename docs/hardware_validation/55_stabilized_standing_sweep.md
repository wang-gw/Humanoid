# Stabilized Standing Sweep

## 목적

단순 joint-space PD에 base roll/pitch와 COM 오차 feedback을 추가했을 때 standing 실패가 줄어드는지 확인한다.

## 실행 명령

```bash
python3 scripts/sweep_stabilized_standing.py
```

## 입력

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml`
- pose: `/home/king0519/projects/Humanoid/configs/quasistatic_standing_pose_inertia_direct_nobase.json`
- 평가 config 수: `432`

## Best Result

- score: `32.000439`
- final base z: `0.178721`
- final roll: `-2.970172` rad
- final pitch: `0.004192` rad
- max |roll|: `3.129725` rad
- max |pitch|: `0.252447` rad
- max qvel norm: `197.272821`
- max contact force: `1858.071654`
- saturation fraction: `0.995025`

## Best Config

- `kp_att`: `8.000000`
- `kd_att`: `0.800000`
- `kcom`: `8.000000`
- `hip_roll_sign`: `-1.000000`
- `ankle_roll_sign`: `-1.000000`
- `hip_pitch_sign`: `1.000000`
- `ankle_pitch_sign`: `1.000000`

## Top Configs

| rank | score | final roll | final pitch | max qvel | max force | kp_att | kd_att | kcom | signs HR/AR/HP/AP |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 32.000439 | -2.970172 | 0.004192 | 197.272821 | 1858.071654 | 8.0 | 0.8 | 8.0 | `-1/-1/+1/+1` |
| 2 | 32.502378 | -1.475242 | 0.199135 | 140.640000 | 1418.084807 | 8.0 | 2.0 | 8.0 | `+1/-1/-1/+1` |
| 3 | 33.160909 | -1.648374 | -0.196657 | 183.791915 | 1876.592177 | 8.0 | 0.8 | 8.0 | `+1/-1/-1/+1` |
| 4 | 33.833396 | -0.839391 | 0.331913 | 160.469382 | 2058.419878 | 16.0 | 0.8 | 8.0 | `+1/-1/+1/+1` |
| 5 | 33.859839 | -1.708696 | 0.461088 | 226.943896 | 1701.784077 | 8.0 | 4.0 | 8.0 | `-1/+1/-1/+1` |
| 6 | 34.097909 | -2.166102 | -0.449516 | 201.426116 | 2077.662596 | 8.0 | 2.0 | 4.0 | `-1/-1/-1/-1` |
| 7 | 34.113908 | -2.129948 | -0.320213 | 168.403108 | 2851.542591 | 8.0 | 0.8 | 4.0 | `+1/+1/+1/-1` |
| 8 | 34.127713 | 2.930056 | -0.169749 | 227.363034 | 1824.020169 | 8.0 | 2.0 | 8.0 | `-1/+1/+1/+1` |

## 산출물

- sweep summary CSV: `/home/king0519/projects/Humanoid/outputs/analysis/stabilized_standing_sweep/stabilizer_sweep_summary.csv`
- sweep summary JSON: `/home/king0519/projects/Humanoid/outputs/analysis/stabilized_standing_sweep/stabilizer_sweep_summary.json`
- best run CSV: `/home/king0519/projects/Humanoid/outputs/analysis/stabilized_standing_sweep/best_stabilized_standing.csv`
- best run plot: `/home/king0519/projects/Humanoid/outputs/analysis/stabilized_standing_sweep/neutral_pd_standing_plot.png`

## 판단

이 stabilizer는 최종 제어기가 아니라 원인 분리용 heuristic이다. 결과가 좋아지면 제어기 부재가 큰 원인이고, 여전히 크게 무너지면 contact/inertia/joint axis 문제 가능성이 남는다.

이번 sweep에서는 pitch는 상당히 줄었지만 roll이 계속 크게 무너졌다. 또한 best config에서도 torque saturation fraction이 `0.995025`로 거의 모든 log sample에서 포화가 발생했다.

따라서 100 Nm 조건에서는 roll 안정화 토크가 충분히 전달되지 않거나, roll 방향 contact/joint/inertial 모델이 아직 맞지 않을 가능성이 크다.
