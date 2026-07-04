# Dynamic Standing Pose Search

## 목적

76번 결과에서 접촉 유지와 torque 비포화는 달성했지만, 2초 동안 roll drift가 계속 누적되었다.

이번 단계에서는 COM/support 중심의 정적 pose search가 아니라, 실제 MuJoCo rollout에서 roll drift가 작아지는 pose를 직접 찾는다.

## 기준 모델과 제어기

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics.xml`
- seed pose: `/home/king0519/projects/Humanoid/configs/quasistatic_standing_pose_actuator_dynamics_contact.json`
- 새 search 스크립트: `/home/king0519/projects/Humanoid/scripts/search_dynamic_standing_pose.py`

Controller:

| 항목 | 값 |
| --- | ---: |
| joint Kp | `20` |
| joint Kd | `12` |
| torque limit | `30 Nm` |
| attitude Kp | `0` |
| attitude Kd | `1` |
| COM K | `1` |
| roll sign | `-1` |
| pitch sign | `-1` |

## Dynamic pose search

실행:

```bash
python3 scripts/search_dynamic_standing_pose.py --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics.xml --seed-pose configs/quasistatic_standing_pose_actuator_dynamics_contact.json --out configs/dynamic_standing_pose_actuator_dynamics_contact.json --out-dir outputs/analysis/dynamic_standing_pose_search_actuator_dynamics_contact --samples 600 --iterations 4 --elite 48 --seed 29 --sigma 0.07 --duration 2.0 --joint-kp 20 --joint-kd 12 --torque-limit 30 --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign -1
```

탐색 조건:

| 항목 | 값 |
| --- | ---: |
| samples | `600` |
| iterations | `4` |
| per iteration | `150` |
| elite | `48` |
| rollout duration | `2.0 s` |
| contact penetration target | `0.001 m` |

Best pose:

```text
configs/dynamic_standing_pose_actuator_dynamics_contact.json
```

Best result from search:

| 항목 | 값 |
| --- | ---: |
| base_z | `0.009134 m` |
| support margin | `0.032774 m` |
| final base_z | `-0.004997 m` |
| final roll | `-0.002286 rad` |
| final pitch | `0.055240 rad` |
| final yaw | `0.220958 rad` |
| max abs roll | `0.065329 rad` |
| max abs pitch | `0.055240 rad` |
| max qvel norm | `0.563220` |
| max contact force | `96.758847 N` |
| max torque | `3.853551 Nm` |
| contact fraction | `1.0` |
| saturation fraction | `0.0` |

## Pose geometry audit

새 pose의 초기 상태:

| 항목 | 값 |
| --- | ---: |
| COM x/y/z | `0.059226 / -0.051689 / 0.291997 m` |
| min support margin | `0.032774 m` |
| lowest contact z | `-0.001000 m` |
| initial contacts | `2` |

초기 접촉은 이전 pose의 `3`개보다 적지만, rollout에서는 2초 동안 contact fraction `1.0`을 유지했다.

## 2초 검증

### PD-only

동일 pose를 단순 PD로 검증했다.

| 항목 | 값 |
| --- | ---: |
| final base_z | `0.000018 m` |
| final roll | `0.071524 rad` |
| final pitch | `0.015457 rad` |
| max qvel norm | `0.566682` |
| max contact force | `96.884126 N` |

### Axis-aware

2초 axis-aware 영상:

```text
outputs/analysis/render_axis_aware_dynamic_pose_actuator_dynamics_2s/axis_aware_standing_render.mp4
```

결과:

| 항목 | 값 |
| --- | ---: |
| final base_z | `-0.004997 m` |
| final roll | `-0.002286 rad` |
| final pitch | `0.055240 rad` |
| max qvel norm | `0.563251` |
| max contact force | `102.005746 N` |
| max torque | `3.859163 Nm` |
| contact frame fraction | `1.0` |

판단:

- 2초 기준으로는 지금까지 가장 좋은 standing 결과다.
- roll drift가 사실상 제거되었다.
- torque saturation도 없다.

## 5초 검증

2초에서 좋았던 `roll_sign=-1, pitch_sign=-1`은 5초에서는 뒤늦게 크게 무너졌다.

따라서 동일 gain에서 부호 4개 조합만 5초로 재평가했다.

Best 5초 조건:

| 항목 | 값 |
| --- | ---: |
| roll sign | `+1` |
| pitch sign | `-1` |
| final base_z | `-0.005931 m` |
| final roll | `-0.106489 rad` |
| final pitch | `0.200554 rad` |
| final yaw | `0.282501 rad` |
| max abs roll | `0.124414 rad` |
| max abs pitch | `0.200554 rad` |
| max qvel norm | `0.873108` |
| max contact force | `101.648505 N` |
| max torque | `5.437417 Nm` |
| contact fraction | `1.0` |
| saturation fraction | `0.0` |

5초 best 영상:

```text
outputs/analysis/render_axis_aware_dynamic_pose_actuator_dynamics_5s_best_sign/axis_aware_standing_render.mp4
```

프레임 관찰:

- `0.0 s`: contacts `2`, roll/pitch 거의 `0`
- `2.528 s`: contacts `4`, roll `+0.12 rad`, pitch 거의 `0`
- `5.0 s`: contacts `2`, roll `-0.11 rad`, pitch `+0.20 rad`

## 현재 판단

이번 단계에서 처음으로 5초 동안 큰 전도 없이 접촉을 유지하는 standing 후보가 나왔다.

좋아진 점:

1. 2초 기준 roll drift는 거의 제거되었다.
2. 5초 기준으로도 contact fraction `1.0`을 유지했다.
3. torque는 `5.44 Nm` 이하로 낮다.
4. saturation은 없다.
5. max qvel도 `0.873` 수준으로 낮다.

남은 문제:

1. 5초 후 pitch가 `0.20 rad`까지 증가한다.
2. base_z가 약 `-6 mm`라 contact penetration/height 기준을 더 다듬어야 한다.
3. 2초 최적 부호와 5초 최적 부호가 다르므로, controller sign/gain을 더 긴 horizon 기준으로 다시 최적화해야 한다.
4. 초기 contact가 `2`개라 foot contact patch가 충분히 균일하지 않을 수 있다.

## 결론

현재 설계는 actuator dynamics가 반영된 모델 기준으로 standing sanity check를 상당 부분 통과하기 시작했다.

다만 아직 RL 보행으로 넘어가기 전에는 다음을 더 해야 한다.

1. 5초 horizon 기준 pose/controller 동시 탐색
2. base_z/contact penetration 보정
3. foot contact를 단일 box에서 다점 pad로 개선할지 판단
4. 실제 AK45 계열 actuator dynamics 값으로 placeholder 보정

## 산출물

- dynamic pose: `/home/king0519/projects/Humanoid/configs/dynamic_standing_pose_actuator_dynamics_contact.json`
- candidate CSV: `/home/king0519/projects/Humanoid/outputs/analysis/dynamic_standing_pose_search_actuator_dynamics_contact/dynamic_pose_search_candidates.csv`
- search summary: `/home/king0519/projects/Humanoid/outputs/analysis/dynamic_standing_pose_search_actuator_dynamics_contact/dynamic_pose_search_summary.json`
- 2초 video: `/home/king0519/projects/Humanoid/outputs/analysis/render_axis_aware_dynamic_pose_actuator_dynamics_2s/axis_aware_standing_render.mp4`
- 5초 best video: `/home/king0519/projects/Humanoid/outputs/analysis/render_axis_aware_dynamic_pose_actuator_dynamics_5s_best_sign/axis_aware_standing_render.mp4`
