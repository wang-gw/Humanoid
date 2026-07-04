# 5초 Dynamic Pose/Controller Search

## 목적

77번에서는 2초 기준 dynamic standing pose를 찾았고, 5초에서도 큰 전도 없이 버티는 후보를 확인했다. 하지만 5초 후 pitch와 base_z 침하가 남아 있었다.

이번 단계에서는 처음부터 5초 horizon을 기준으로 pose를 다시 찾고, roll/pitch/base_z를 더 강하게 비용에 반영한다.

## 기준 모델

```text
envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics.xml
```

현재 이 모델은 최종 하드웨어 모델이 아니라 actuator dynamics placeholder가 들어간 검증 모델이다.

## Search 조건

Seed pose:

```text
configs/dynamic_standing_pose_actuator_dynamics_contact.json
```

Output pose:

```text
configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json
```

실행:

```bash
python3 scripts/search_dynamic_standing_pose.py --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics.xml --seed-pose configs/dynamic_standing_pose_actuator_dynamics_contact.json --out configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json --out-dir outputs/analysis/dynamic_standing_pose_search_actuator_dynamics_5s --samples 360 --iterations 3 --elite 36 --seed 41 --sigma 0.045 --duration 5.0 --joint-kp 20 --joint-kd 12 --torque-limit 30 --kp-att 0 --kd-att 1 --kcom 1 --roll-sign 1 --pitch-sign -1 --extra-roll-weight 8 --extra-pitch-weight 8 --extra-yaw-weight 1 --base-z-min -0.003 --base-z-penalty 30
```

Controller:

| 항목 | 값 |
| --- | ---: |
| joint Kp | `20` |
| joint Kd | `12` |
| torque limit | `30 Nm` |
| attitude Kp | `0` |
| attitude Kd | `1` |
| COM K | `1` |
| roll sign | `+1` |
| pitch sign | `-1` |

Search:

| 항목 | 값 |
| --- | ---: |
| samples | `360` |
| iterations | `3` |
| per iteration | `120` |
| elite | `36` |
| sigma | `0.045` |
| rollout duration | `5.0 s` |
| contact penetration target | `0.001 m` |

추가 score weight:

| 항목 | 값 |
| --- | ---: |
| extra roll weight | `8` |
| extra pitch weight | `8` |
| extra yaw weight | `1` |
| base_z_min | `-0.003 m` |
| base_z penalty | `30` |

## Best search 결과

| 항목 | 값 |
| --- | ---: |
| base_z | `0.018914 m` |
| support margin | `0.011642 m` |
| final base_z | `-0.001903 m` |
| final roll | `0.004699 rad` |
| final pitch | `-0.007412 rad` |
| final yaw | `0.052937 rad` |
| max abs roll | `0.063172 rad` |
| max abs pitch | `0.013797 rad` |
| max qvel norm | `0.574186` |
| max contact force | `102.595089 N` |
| max torque | `3.780377 Nm` |
| contact fraction | `1.0` |
| saturation fraction | `0.0` |

Joint targets:

| joint | target rad |
| --- | ---: |
| `left_hip_roll` | `0.054251` |
| `left_hip_pitch` | `0.003927` |
| `left_knee_pitch` | `0.025379` |
| `left_ankle_pitch` | `0.199447` |
| `left_ankle_roll` | `-0.109303` |
| `right_hip_roll` | `0.283405` |
| `right_hip_pitch` | `0.094556` |
| `right_knee_pitch` | `0.137822` |
| `right_ankle_pitch` | `-0.144600` |
| `right_ankle_roll` | `-0.072594` |

## Pose geometry audit

| 항목 | 값 |
| --- | ---: |
| COM x/y/z | `0.060909 / -0.048030 / 0.302935 m` |
| min support margin | `0.011642 m` |
| lowest contact z | `-0.001000 m` |
| initial contacts | `1` |

주의:

- 정적 support margin은 이전 pose보다 작다.
- 초기 contact도 `1`개로 적다.
- 하지만 dynamic rollout에서는 접촉이 빠르게 안정되고, 5초/10초 모두 contact fraction `1.0`을 유지했다.

## 5초 검증

영상:

```text
outputs/analysis/render_axis_aware_dynamic_pose_actuator_dynamics_5s_optimized/axis_aware_standing_render.mp4
```

결과:

| 항목 | 값 |
| --- | ---: |
| final base_z | `-0.001903 m` |
| final roll | `0.004699 rad` |
| final pitch | `-0.007412 rad` |
| final yaw | `0.052937 rad` |
| final qvel norm | `0.024485` |
| max qvel norm | `0.574290` |
| max contact force | `104.029033 N` |
| max torque | `3.909655 Nm` |
| contacts final | `4` |
| contact frame fraction | `1.0` |

프레임 관찰:

- `5.0 s`: roll/pitch가 거의 0에 가깝고, 발 접촉이 유지된다.

## 10초 검증

5초 pose를 10초로 늘려 검증했다. 10초 기준으로는 부호 조합을 다시 평가했고, 다음 조건이 가장 좋았다.

| 항목 | 값 |
| --- | ---: |
| roll sign | `-1` |
| pitch sign | `+1` |

영상:

```text
outputs/analysis/render_axis_aware_dynamic_pose_actuator_dynamics_10s_optimized_best_sign/axis_aware_standing_render.mp4
```

결과:

| 항목 | 값 |
| --- | ---: |
| final base_z | `-0.003692 m` |
| final roll | `0.007634 rad` |
| final pitch | `-0.041316 rad` |
| final yaw | `0.024149 rad` |
| final qvel norm | `0.010247` |
| max qvel norm | `0.587883` |
| max contact force | `105.097354 N` |
| max torque | `3.817249 Nm` |
| contacts final | `5` |
| contact frame fraction | `1.0` |
| saturation fraction | `0.0` |

프레임 관찰:

- `0.0 s`: contacts `1`, 초기에는 한쪽 접촉 위주
- `5.024 s`: contacts `5`, roll `+0.01`, pitch `-0.03`
- `10.0 s`: contacts `5`, roll `+0.01`, pitch `-0.04`

## 현재 판단

이번 결과는 standing sanity check 관점에서 큰 진전이다.

통과에 가까운 항목:

1. 10초 동안 큰 전도 없이 유지한다.
2. 접촉 frame fraction이 `1.0`이다.
3. torque saturation이 없다.
4. 최대 토크가 `4 Nm` 미만이다.
5. 최종 roll/pitch가 작다.
6. qvel이 매우 낮게 수렴한다.

아직 확정하면 안 되는 항목:

1. actuator dynamics 값은 placeholder다.
2. 초기 contact가 `1`개라 contact patch가 균일하지 않다.
3. 5초 최적 sign과 10초 최적 sign이 달라 controller 구조가 아직 임시적이다.
4. standing은 가능해졌지만, weight shift나 stepping은 아직 검증하지 않았다.

## 결론

현재 설계는 actuator dynamics placeholder와 dynamic pose optimization을 적용했을 때, MuJoCo에서 10초 standing sanity check를 통과하는 수준까지 왔다.

따라서 "하드웨어 형상이 RL로 절대 걸을 수 없는 상태"라고 보기는 어렵다. 다만 이 결론은 최종 모터/감속기 모델이 아니라 placeholder actuator dynamics 기준이다.

다음 단계는 다음 중 하나다.

1. 실제 AK45-36 / AK45-10 actuator dynamics 값을 반영한다.
2. 현재 모델 기준으로 weight shift test를 수행한다.
3. 10초 standing pose를 기준으로 RL 초기 state/reference로 사용할 수 있는지 검토한다.

검증 순서상 다음은 `weight shift`가 적절하다. standing이 가능하더라도 좌우로 하중을 옮기지 못하면 보행 RL로 넘어갈 수 없다.

## 산출물

- 5초 optimized pose: `/home/king0519/projects/Humanoid/configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json`
- search candidates: `/home/king0519/projects/Humanoid/outputs/analysis/dynamic_standing_pose_search_actuator_dynamics_5s/dynamic_pose_search_candidates.csv`
- search summary: `/home/king0519/projects/Humanoid/outputs/analysis/dynamic_standing_pose_search_actuator_dynamics_5s/dynamic_pose_search_summary.json`
- 5초 video: `/home/king0519/projects/Humanoid/outputs/analysis/render_axis_aware_dynamic_pose_actuator_dynamics_5s_optimized/axis_aware_standing_render.mp4`
- 10초 video: `/home/king0519/projects/Humanoid/outputs/analysis/render_axis_aware_dynamic_pose_actuator_dynamics_10s_optimized_best_sign/axis_aware_standing_render.mp4`
- 10초 score summary: `/home/king0519/projects/Humanoid/outputs/analysis/axis_aware_dynamic_pose_actuator_dynamics_10s_optimized_score/axis_aware_sweep_summary.json`
