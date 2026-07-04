# Quasi-static Pose PD Probe

## 목적

53번에서 찾은 준정적 standing 후보를 dynamics probe에 넣어, neutral pose 실패가 단순히 `0 rad` 자세 문제였는지 확인한다.

## 입력

| 항목 | 값 |
| --- | --- |
| 모델 | `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml` |
| pose | `configs/quasistatic_standing_pose_inertia_direct_nobase.json` |
| controller | joint-space PD |
| Kp / Kd | `60 / 4` |
| torque limit | `100 Nm` |

## Pose 후보 요약

| 항목 | 값 |
| --- | ---: |
| base z | `0.000266 m` |
| COM world | `[0.068316, -0.055055, 0.285901]` |
| support margins | `[0.101918, 0.101130, 0.063973, 0.062807]` |
| foot height diff | `0.000463 m` |
| foot normal penalty | `0.000078` |
| COM center penalty | `0.000100` |

Joint targets:

| joint | target rad |
| --- | ---: |
| `left_hip_roll` | 0.159164 |
| `left_hip_pitch` | -0.158644 |
| `left_knee_pitch` | 0.086996 |
| `left_ankle_pitch` | 0.002557 |
| `left_ankle_roll` | -0.045124 |
| `right_hip_roll` | 0.223145 |
| `right_hip_pitch` | 0.065207 |
| `right_knee_pitch` | -0.046965 |
| `right_ankle_pitch` | -0.097717 |
| `right_ankle_roll` | -0.227900 |

## 실행 명령

```bash
python3 scripts/probe_pd_standing.py \
  --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_direct_nobase_contact.xml \
  --pose-json configs/quasistatic_standing_pose_inertia_direct_nobase.json \
  --out-dir outputs/analysis/pd_standing_inertia_direct_nobase_quasistatic_pose \
  --doc-dir docs/hardware_validation/tmp \
  --doc-name inertia_direct_nobase_quasistatic_pose.md

python3 scripts/plot_pd_standing.py \
  --csv outputs/analysis/pd_standing_inertia_direct_nobase_quasistatic_pose/neutral_pd_standing.csv \
  --out-dir outputs/analysis/pd_standing_inertia_direct_nobase_quasistatic_pose
```

## 결과 비교

| 모델/pose | final base z | final roll | final pitch | max qvel norm | max contact force |
| --- | ---: | ---: | ---: | ---: | ---: |
| mass baseline neutral | 0.131486 | -2.453577 | 0.150364 | 857.401243 | 4711.977344 |
| inertia neutral | 0.026609 | 2.588957 | 1.212512 | 266.242408 | 1303.468953 |
| inertia quasi-static pose | 0.132326 | 3.023535 | 0.613897 | 195.292698 | 1951.860573 |

## Ankle RMS Torque

| actuator | mass baseline neutral | inertia neutral | inertia quasi-static pose |
| --- | ---: | ---: | ---: |
| `motor_left_ankle_pitch` | 95.848049 | 84.464979 | 94.715753 |
| `motor_left_ankle_roll` | 97.083598 | 84.870726 | 95.773411 |
| `motor_right_ankle_pitch` | 93.344675 | 50.794615 | 73.946256 |
| `motor_right_ankle_roll` | 93.298375 | 86.609677 | 96.890007 |

## 산출물

- pose config: `configs/quasistatic_standing_pose_inertia_direct_nobase.json`
- summary: `outputs/analysis/pd_standing_inertia_direct_nobase_quasistatic_pose/neutral_pd_standing_summary.json`
- plot: `outputs/analysis/pd_standing_inertia_direct_nobase_quasistatic_pose/neutral_pd_standing_plot.png`

## 판단

준정적 pose는 COM support margin과 양발 높이 조건을 개선했다. dynamics probe에서도 max qvel norm은 `266.242408 -> 195.292698`로 더 줄었다.

하지만 final roll이 `3.023535 rad`, final pitch가 `0.613897 rad`까지 무너져 standing 성공은 아니다. 즉, 실패 원인을 단순히 “neutral `0 rad` pose가 안 좋아서”라고만 볼 수 없다.

현재 남은 원인 후보는 다음과 같다.

- floating-base standing에 joint-space PD만 사용하는 controller 한계
- foot contact가 box 하나라 roll/pitch 안정 모멘트가 충분히 재현되지 않는 문제
- `base_link`와 hip-roll actuator 계열 inertial frame 미확정
- ankle roll/pitch 축 또는 sign이 균형 제어에 기대한 방향과 다를 가능성

## 다음 단계

다음에는 open-loop pose 유지가 아니라, base roll/pitch와 COM 오차를 ankle/hip torque로 되먹임하는 간단한 stabilizer를 추가해본다. 이 stabilizer에서도 버티지 못하면 하드웨어 형상/contact/inertia 쪽 문제 가능성이 더 커진다.

