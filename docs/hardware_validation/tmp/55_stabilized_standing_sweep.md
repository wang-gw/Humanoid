# Stabilized Standing Sweep

## 목적

단순 joint-space PD에 base roll/pitch와 COM 오차 feedback을 추가했을 때 standing 실패가 줄어드는지 확인한다.

## 실행 명령

```bash
python3 scripts/sweep_stabilized_standing.py
```

## 입력

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml`
- pose: `/home/king0519/projects/Humanoid/configs/quasistatic_standing_pose_toeheel_z_step_visual_contact.json`
- 평가 config 수: `432`

## Best Result

- score: `32.313245`
- final base z: `-0.017862`
- final roll: `-2.041031` rad
- final pitch: `-0.142492` rad
- max |roll|: `3.133306` rad
- max |pitch|: `0.282258` rad
- max qvel norm: `215.187207`
- max contact force: `1582.382810`
- saturation fraction: `0.990050`

## Best Config

- `kp_att`: `8.000000`
- `kd_att`: `2.000000`
- `kcom`: `0.000000`
- `hip_roll_sign`: `-1.000000`
- `ankle_roll_sign`: `-1.000000`
- `hip_pitch_sign`: `1.000000`
- `ankle_pitch_sign`: `1.000000`

## Top Configs

| rank | score | final roll | final pitch | max qvel | max force | kp_att | kd_att | kcom | signs HR/AR/HP/AP |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 32.313245 | -2.041031 | -0.142492 | 215.187207 | 1582.382810 | 8.0 | 2.0 | 0.0 | `-1/-1/+1/+1` |
| 2 | 32.518713 | -2.061416 | 0.265628 | 211.434963 | 1807.916724 | 8.0 | 0.8 | 8.0 | `+1/-1/+1/+1` |
| 3 | 32.610432 | -1.273232 | 0.125228 | 205.774995 | 1444.144414 | 16.0 | 2.0 | 8.0 | `-1/+1/-1/+1` |
| 4 | 33.086701 | -1.728519 | -0.111012 | 174.483499 | 1326.971010 | 8.0 | 0.8 | 0.0 | `-1/+1/-1/-1` |
| 5 | 33.185622 | -2.085144 | 0.091699 | 222.317809 | 1889.915342 | 16.0 | 0.8 | 4.0 | `-1/-1/-1/-1` |
| 6 | 33.269058 | -2.297695 | 0.252143 | 158.841205 | 1667.027689 | 8.0 | 2.0 | 0.0 | `-1/+1/+1/+1` |
| 7 | 33.345088 | -1.861392 | 0.312112 | 163.079525 | 1484.651906 | 8.0 | 4.0 | 8.0 | `-1/-1/-1/-1` |
| 8 | 33.639391 | -1.724168 | -0.068116 | 138.184687 | 1704.327608 | 8.0 | 2.0 | 8.0 | `+1/-1/-1/-1` |

## 산출물

- sweep summary CSV: `/home/king0519/projects/Humanoid/outputs/analysis/stabilized_standing_toeheel_z_step_visual_contact_pose/stabilizer_sweep_summary.csv`
- sweep summary JSON: `/home/king0519/projects/Humanoid/outputs/analysis/stabilized_standing_toeheel_z_step_visual_contact_pose/stabilizer_sweep_summary.json`
- best run CSV: `/home/king0519/projects/Humanoid/outputs/analysis/stabilized_standing_toeheel_z_step_visual_contact_pose/best_stabilized_standing.csv`

## 판단

이 stabilizer는 최종 제어기가 아니라 원인 분리용 heuristic이다. 결과가 좋아지면 제어기 부재가 큰 원인이고, 여전히 크게 무너지면 contact/inertia/joint axis 문제 가능성이 남는다.
