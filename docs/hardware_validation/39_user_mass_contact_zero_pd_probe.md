# 사용자 제공 Mass/Contact + 0 rad Standing PD Probe

## 목적

사용자가 제공한 standing pose, foot contact box, mass property를 반영한 모델에서 0 rad 직립 자세가 단순 PD standing을 통과하는지 확인한다.

## 실행 명령

```bash
python3 scripts/probe_pd_standing.py --model envs/robots/urdf_f_link/URDF_F_link_user_mass_contact.xml --base-z 0.04228416147400482 --out-dir outputs/analysis/pd_standing_user_mass_contact_zero --doc-dir docs/hardware_validation --doc-name 39_user_mass_contact_zero_pd_probe.md
python3 scripts/plot_pd_standing.py --csv outputs/analysis/pd_standing_user_mass_contact_zero/neutral_pd_standing.csv --out-dir outputs/analysis/pd_standing_user_mass_contact_zero
```

## 조건

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_mass_contact.xml`
- 목표 joint pose: 전체 revolute joint `0.0 rad`
- base z: `0.042284`
- Kp: `60.0`
- Kd: `4.0`
- torque limit: `+/-100.0 Nm`
- 총질량: `9.211400 kg`

## Geometry 선행 결과

사용자 contact 값을 현재 MJCF body frame에 그대로 적용했을 때:

- left sole world pos: `[-0.00975, 0.0385, 0.077284]`
- right sole world pos: `[-0.16425, 0.0385, 0.127284]`
- 좌우 contact 높이 차이: 약 `0.05 m`
- COM inside support x: `False`
- COM inside support y: `False`

## PD 결과

- 최종 base z: `1.707725`
- 최종 roll: `-2.687330` rad
- 최종 pitch: `-0.596469` rad
- 최대 qvel norm: `857.594066`
- 최대 contact normal force: `3404.384879`
- 관측 최대 actuator torque: `100.000000` Nm

## 조인트별 토크 요약

| actuator | 최대 절대 토크 Nm | RMS 토크 Nm |
| --- | ---: | ---: |
| `motor_left_hip_roll` | 68.630974 | 25.937245 |
| `motor_left_hip_pitch` | 70.854517 | 22.330229 |
| `motor_left_knee_pitch` | 100.000000 | 27.712815 |
| `motor_left_ankle_pitch` | 100.000000 | 93.790830 |
| `motor_left_ankle_roll` | 100.000000 | 96.385776 |
| `motor_right_hip_roll` | 65.041889 | 28.755294 |
| `motor_right_hip_pitch` | 59.888192 | 16.349291 |
| `motor_right_knee_pitch` | 73.231148 | 23.508230 |
| `motor_right_ankle_pitch` | 100.000000 | 96.569436 |
| `motor_right_ankle_roll` | 100.000000 | 94.407758 |

## 산출물

- CSV: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_user_mass_contact_zero/neutral_pd_standing.csv`
- JSON: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_user_mass_contact_zero/neutral_pd_standing_summary.json`
- Plot: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_user_mass_contact_zero/neutral_pd_standing_plot.png`

## 해석

0 rad pose를 standing pose로 두어도 현재 MJCF frame에 사용자 contact 값을 그대로 적용한 모델은 안정 standing을 만들지 못한다.

특히 좌우 contact 높이가 50 mm 차이 나는 것이 확인되었으므로, 제공된 foot contact center는 Fusion local frame 기준이고 현재 MJCF foot body frame과 직접 일치하지 않는 것으로 보는 것이 타당하다.
